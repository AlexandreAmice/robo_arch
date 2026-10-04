"""Build fixed-base USD scenes from shared physical declarations."""

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
from robo_arch.core.worlds.isaac.config import IsaacPhysics, IsaacWorld
from robo_arch.core.worlds.isaac.objects import add_objects
from robo_arch.core.worlds.isaac.urdf import add_to_stage
from robo_arch.core.worlds.urdf import compose


@dataclass
class IsaacScene:
    """Unattached stage; the caller keeps converted assets alive through execution."""

    stage: Any
    physics: Any
    roots: dict[str, str]
    devices: Devices
    definitions: DeviceDefinitions
    configuration: SceneConfiguration


def apply_physics(scene, config: IsaacPhysics):
    """Author native scene settings before attaching the stage to PhysX.

    CPU uses the native MBP broadphase. GPU dynamics uses the GPU broadphase on
    CUDA device zero. Articulation solver limits remain owned by the robot asset.
    """
    from pxr import PhysxSchema

    physics = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    physics.CreateSolverTypeAttr(config.solver.upper())
    physics.CreateEnableGPUDynamicsAttr(config.device == "cuda:0")
    physics.CreateGpuFoundLostAggregatePairsCapacityAttr(
        config.gpu_found_lost_aggregate_pairs_capacity
    )
    physics.CreateBroadphaseTypeAttr("GPU" if config.device == "cuda:0" else "MBP")
    return physics


def add_ground(stage: Any) -> None:
    """Author a static z=0 collision plane and separate 10 m square visual.

    PhysX treats UsdGeom.Plane as an infinite collision plane. Hydra renders the
    finite mesh instead, matching Isaac's native ground-plane representation.
    Material values are the ground's own nominal contact assumptions.
    """
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
    """Author physics and assets; requires Kit startup but does not step physics.

    The caller owns the conversion directory and must retain it until the stage
    and its PhysX/viewer consumers have been released.
    """
    if not isinstance(config, IsaacWorld):
        raise ValueError("Isaac scene construction requires IsaacWorld")
    definitions = load_definitions(scene, config.type)
    devices = resolve_devices(scene)
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    physics_scene = UsdPhysics.Scene.Define(stage, "/physics")
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    physics = apply_physics(physics_scene, config.physics)
    if config.ground:
        add_ground(stage)
    roots = {}
    for robot in devices.robots:
        X_WB = base_pose(robot).GetAsMatrix4()
        destination = directory / robot.name
        destination.mkdir(parents=True)
        urdf = destination / "assembly.urdf"
        compose(robot, devices.sensors, definitions, urdf)
        roots[robot.name] = add_to_stage(
            stage,
            name=robot.name,
            X_WB=X_WB,
            directory=destination / "usd",
            urdf=urdf,
        )
    add_objects(stage, scene, definitions)
    return IsaacScene(stage, physics, roots, devices, definitions, scene)


def initialize_scene(scene: IsaacScene, view) -> tuple[dict, dict, dict]:
    """Bind initialized PhysX articulations and apply configured initial state."""
    devices, definitions = scene.devices, scene.definitions
    arms, sensors, efforts = {}, {}, {}
    indices = np.array([0], dtype=np.uint32)
    for robot in devices.robots:
        arm = view.create_articulation_view(scene.roots[robot.name])
        definition = definitions.robots[robot.model]
        if arm.count != 1 or not arm.shared_metatype.fixed_base:
            raise ValueError(f"Expected a fixed-base articulation for {robot.name}")
        if tuple(arm.shared_metatype.dof_names) != definition.joints:
            raise ValueError(f"Isaac joint order differs for {robot.name}")
        initial = (
            definition.default_positions
            if robot.initial_positions is None
            else robot.initial_positions
        )
        q = np.asarray(initial, dtype=np.float32).reshape(1, -1)
        limits = arm.get_dof_limits()[0]
        if q.shape != (1, len(definition.joints)) or not np.isfinite(q).all():
            raise ValueError(f"Invalid initial positions for {robot.name}")
        if not np.all((q[0] >= limits[:, 0]) & (q[0] <= limits[:, 1])):
            raise ValueError(f"Initial positions exceed limits for {robot.name}")
        arm.set_dof_positions(q, indices)
        arm.set_dof_velocities(np.zeros_like(q), indices)
        arms[robot.name] = arm
        efforts[robot.name] = np.zeros(len(definition.joints))
    for sensor in devices.sensors if scene.configuration.sensors_enabled else ():
        parent = sensor.parent.rsplit("/", 1)[0]
        adapter = load_device_module("sensors", sensor.model, "isaac")
        sensors[sensor.name] = adapter.bind(arms[parent], name=sensor.name)
    return arms, sensors, efforts
