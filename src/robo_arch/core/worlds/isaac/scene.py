"""Assemble shared declarations into an Isaac Lab scene using configurable physics."""

import math
import xml.etree.ElementTree as ET
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np

from robo_arch.core.config.declarations import SceneConfiguration
from robo_arch.core.worlds.assembly import Devices, base_pose, resolve_devices
from robo_arch.core.worlds.devices import (
    DeviceDefinitions,
    load_definitions,
    load_device_module,
)
from robo_arch.core.worlds.isaac.backend import validate_observations
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.objects import add_objects
from robo_arch.core.worlds.isaac.urdf import add_to_stage
from robo_arch.core.worlds.urdf import compose


@dataclass
class IsaacScene:
    """Lab owns scene buffers; the caller retains converted files until shutdown."""

    native: Any
    simulation: Any
    roots: dict[str, str]
    devices: Devices
    definitions: DeviceDefinitions
    configuration: SceneConfiguration
    world: IsaacWorld
    observations: dict[str, Callable[[], np.ndarray]]

    @property
    def stage(self) -> Any:
        return self.simulation.stage


def add_ground(stage: Any, *, enabled: bool = True) -> None:
    """Author a static z=0 collision plane and separate 10 m square visual.

    PhysX treats UsdGeom.Plane as an infinite collision plane. Hydra renders the
    finite mesh instead, matching Isaac's native ground-plane representation.
    Material values are the ground's own nominal contact assumptions.
    """
    if not enabled:
        return
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    # Declared device names start with letters, leaving this namespace reserved.
    UsdGeom.Xform.Define(stage, "/_world")
    UsdGeom.Xform.Define(stage, "/_world/ground")
    plane = UsdGeom.Plane.Define(stage, "/_world/ground/collision")
    plane.CreateAxisAttr(UsdGeom.Tokens.z)
    plane.CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())

    material = UsdShade.Material.Define(stage, "/_world/ground/material")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(0.8)
    physics.CreateDynamicFrictionAttr(0.6)
    physics.CreateRestitutionAttr(0.0)
    UsdShade.MaterialBindingAPI.Apply(plane.GetPrim()).Bind(
        material, materialPurpose="physics"
    )

    visual = UsdGeom.Mesh.Define(stage, "/_world/ground/visual")
    visual.CreatePointsAttr(
        [(-5.0, -5.0, 0.0), (5.0, -5.0, 0.0), (5.0, 5.0, 0.0), (-5.0, 5.0, 0.0)]
    )
    visual.CreateFaceVertexCountsAttr([4])
    visual.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    visual.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    visual.CreateDoubleSidedAttr(True)
    visual.CreateDisplayColorAttr([Gf.Vec3f(0.55, 0.57, 0.60)])


def build_scene(
    scene: SceneConfiguration, config: IsaacWorld, *, directory: Path
) -> IsaacScene:
    """Create a Lab simulation and clone the complete physical assembly.

    PhysX parses USD copies; Newton replicates a native model plus display USD.
    Ordinary robots use their declared assets, without model-specific factories.
    """
    if not isinstance(config, IsaacWorld):
        raise ValueError("Isaac scene construction requires IsaacWorld")
    definitions = load_definitions(scene, config.type)
    devices = resolve_devices(scene)
    validate_observations(
        config.physics,
        tuple(sensor.name for sensor in devices.sensors)
        if scene.sensors_enabled
        else (),
    )
    import torch
    from isaaclab import cloner
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.sim import SimulationCfg, SimulationContext
    from pxr import Gf, UsdGeom, UsdPhysics

    from robo_arch.core.worlds.isaac.backend import cloning_contexts, physics_config

    simulation = SimulationContext(
        SimulationCfg(
            dt=config.physics.time_step,
            device=config.physics.device,
            physics=physics_config(config.physics),
            use_fabric=False,
            create_stage_in_memory=False,
            use_newton_actuators=False,
            visualizer_cfgs=[],
        )
    )
    # Native buffers serve control/sensors. Storm publishes USD only at its
    # display cadence; Lab's no-Fabric default would otherwise write every step.
    for setting in ("updateToUsd", "updateVelocitiesToUsd", "updateForceSensorsToUsd"):
        simulation.set_setting(f"/physics/{setting}", False)
    native = InteractiveScene(
        InteractiveSceneCfg(
            num_envs=config.num_envs,
            env_spacing=config.env_spacing,
            replicate_physics=False,
        )
    )
    stage = simulation.stage
    add_ground(stage, enabled=config.ground)
    prototype = "/World/envs/env_0"
    UsdGeom.Xform.Define(stage, prototype)
    roots = {}
    for robot in devices.robots:
        destination = directory / robot.name
        destination.mkdir(parents=True)
        urdf = destination / "assembly.urdf"
        compose(robot, devices.sensors, definitions, urdf)
        # Removing imported position drives also removes their max-force field.
        # Preserve physical effort limits explicitly for both effort backends.
        model = ET.parse(urdf).getroot()
        effort_limits = {}
        for joint_name in definitions.robots[robot.model].joints:
            limit = model.find(f"joint[@name='{joint_name}']/limit")
            if limit is None or "effort" not in limit.attrib:
                raise ValueError(f"Missing URDF effort limit for {joint_name}")
            effort = float(limit.attrib["effort"])
            if not math.isfinite(effort) or effort <= 0:
                raise ValueError(f"Invalid URDF effort limit for {joint_name}")
            effort_limits[joint_name] = effort
        root = add_to_stage(
            stage,
            name=prototype.lstrip("/") + "/" + robot.name,
            X_WB=base_pose(robot).GetAsMatrix4(),
            directory=destination / "usd",
            urdf=urdf,
        )
        roots[robot.name] = root.replace(prototype, "/World/envs/env_.*", 1)
        definition = definitions.robots[robot.model]
        native.articulations[robot.name] = Articulation(
            ArticulationCfg(
                prim_path=roots[robot.name],
                spawn=None,
                cloning_contexts=cloning_contexts(config.physics),
                joint_ordering=definition.joints,
                actuators={
                    "effort": ImplicitActuatorCfg(
                        joint_names_expr=[".*"],
                        stiffness=0.0,
                        damping=0.0,
                        joint_effort_limit=effort_limits,
                    )
                },
            )
        )
    add_objects(stage, scene, definitions, root=prototype)
    positions = torch.zeros((config.num_envs, 3), device=config.physics.device)
    ids = torch.arange(config.num_envs, device=positions.device)
    columns = math.ceil(math.sqrt(config.num_envs))
    positions[:, 0] = (
        ids % columns if config.env_layout == "grid" else ids
    ) * config.env_spacing
    if config.env_layout == "grid":
        positions[:, 1] = (ids // columns) * config.env_spacing
    plan = cloner.clone_plan_from_env_0(
        prototype,
        "/World/envs/env_{}",
        config.num_envs,
        config.physics.device,
        positions,
        global_paths=("/_world/ground",) if config.ground else (),
    )
    cloner.replicate(
        plan, stage=stage, replicate_physics=config.physics.backend == "newton"
    )
    # Fixed joints anchored to world store a world-space parent anchor. USD root
    # translation alone does not relocate that anchor when environments clone.
    for env, position in enumerate(positions.cpu().tolist()):
        offset = Gf.Vec3f(*position)
        for root in roots.values():
            joint = UsdPhysics.FixedJoint.Get(
                stage, root.replace("env_.*", f"env_{env}")
            )
            if not joint or joint.GetBody0Rel().GetTargets():
                raise ValueError(f"Expected an articulation fixed to world at {root}")
            joint.GetLocalPos0Attr().Set(joint.GetLocalPos0Attr().Get() + offset)
    if config.physics.backend == "physx":
        native.filter_collisions(
            global_prim_paths=["/_world/ground"] if config.ground else []
        )
    observations = {}
    for sensor in devices.sensors if scene.sensors_enabled else ():
        parent = sensor.parent.rsplit("/", 1)[0]
        adapter = load_device_module("sensors", sensor.model, "isaac")
        native.sensors[sensor.name] = adapter.create(roots[parent])
        observations[sensor.name] = adapter.bind(
            native.sensors[sensor.name], name=sensor.name
        )
    return IsaacScene(
        native, simulation, roots, devices, definitions, scene, config, observations
    )


def initialize_scene(scene: IsaacScene) -> dict[str, np.ndarray]:
    """Validate Lab articulation identity and declared initial joint positions."""
    initial = {}
    for robot in scene.devices.robots:
        arm = scene.native.articulations[robot.name]
        definition = scene.definitions.robots[robot.model]
        if arm.num_instances != scene.world.num_envs or not arm.is_fixed_base:
            raise ValueError(f"Expected fixed-base articulations for {robot.name}")
        if tuple(arm.joint_names) != definition.joints:
            raise ValueError(f"Isaac joint order differs for {robot.name}")
        q = np.asarray(
            definition.default_positions
            if robot.initial_positions is None
            else robot.initial_positions,
            dtype=np.float32,
        )
        limits = arm.data.joint_pos_limits.torch.cpu().numpy()
        if q.shape != (len(definition.joints),) or not np.isfinite(q).all():
            raise ValueError(f"Invalid initial positions for {robot.name}")
        if not np.all((q >= limits[..., 0]) & (q <= limits[..., 1])):
            raise ValueError(f"Initial positions exceed limits for {robot.name}")
        initial[robot.name] = q
    return initial
