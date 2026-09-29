"""Run a fixed-base arm with explicit external effort feedback in GPU PhysX."""

import math
import time
import xml.etree.ElementTree as ET
from collections.abc import Callable
from importlib.resources import as_file, files
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from robo_arch.core.config.declarations import Pose, RunConfiguration
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import DeviceDefinitions, load_device_module


def _transform(pose: Pose) -> np.ndarray:
    result = np.eye(4)
    roll, pitch, yaw = pose.rpy
    sr, sp, sy = np.sin([roll, pitch, yaw])
    cr, cp, cy = np.cos([roll, pitch, yaw])
    result[:3, :3] = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
    result[:3, 3] = pose.translation
    return result


def _add_objects(stage, run: RunConfiguration, definitions: DeviceDefinitions) -> None:
    """Support the current fixed single-link SDF box fixtures explicitly."""
    from pxr import Gf, UsdGeom, UsdPhysics

    for obj in run.objects:
        definition = definitions.objects[obj.model]
        data = files(definition.package).joinpath(definition.resource).read_text()
        sdf = ET.fromstring(data)
        links = sdf.findall("model/link")
        size = sdf.find("model/link/visual/geometry/box/size")
        collision = sdf.find("model/link/collision/geometry/box/size")
        if (
            len(links) != 1
            or size is None
            or collision is None
            or size.text != collision.text
            or sdf.findall(".//pose")
        ):
            raise ValueError("Isaac fixtures currently require one unoffset SDF box")
        dimensions = np.asarray([float(v) for v in size.text.split()])
        if dimensions.shape != (3,) or not np.all(dimensions > 0):
            raise ValueError("Invalid box dimensions")
        box = UsdGeom.Cube.Define(stage, "/objects/" + obj.name)
        box.CreateSizeAttr(1.0)
        transform = _transform(obj.pose)
        transform[:3, :3] = transform[:3, :3] @ np.diag(dimensions)
        box.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())


def run_scene(
    run: RunConfiguration,
    definitions: DeviceDefinitions,
    command: Callable[[np.ndarray, float], np.ndarray],
    *,
    trace_path: Path | None = None,
) -> dict:
    """Sample [q,v] and apply effort in registered joint order every physics step.

    This initial adapter accepts one fixed-base robot and no sensors. NumPy
    state/commands cross the GPU boundary at each step; this is scalar control,
    not a tensor-efficient training loop. Partial measured traces survive errors.
    """
    devices = resolve_devices(run)
    if run.world != "isaac" or len(devices.robots) != 1 or devices.sensors:
        raise ValueError("Isaac runner requires one robot and explicitly no sensors")
    from isaacsim import SimulationApp

    started = time.monotonic()
    times, positions = [], []
    experience = files(__package__).joinpath("physics.kit")
    with as_file(experience) as path:
        app = SimulationApp(
            {
                "headless": True,
                "multi_gpu": False,
                "renderer": "MinimalRendering",
                "create_new_stage": False,
                "fast_shutdown": False,
            },
            experience=str(path),
        )
    try:
        import omni.physics.tensors as tensors
        from omni.physx import get_physx_simulation_interface
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdUtils

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        scene = UsdPhysics.Scene.Define(stage, "/physics")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        physics = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physics.CreateEnableGPUDynamicsAttr(True)
        physics.CreateBroadphaseTypeAttr("GPU")
        robot = devices.robots[0]
        definition = definitions.robots[robot.model]
        X_WB = np.eye(4)
        for pose in robot.poses:
            X_WB = X_WB @ _transform(pose)
        with TemporaryDirectory(prefix="robo_arch_isaac_") as directory:
            adapter = load_device_module("robots", robot.model, "isaac")
            root = adapter.add_to_stage(
                stage, name=robot.name, X_WB=X_WB, directory=Path(directory)
            )
            _add_objects(stage, run, definitions)
            cache = UsdUtils.StageCache.Get()
            stage_id = cache.Insert(stage).ToLongInt()
            simulation = get_physx_simulation_interface()
            simulation.attach_stage(stage_id)
            try:
                # Initialize PhysX before obtaining tensor views; then reset q,v.
                simulation.simulate(run.time_step, 0.0)
                simulation.fetch_results()
                view = tensors.create_simulation_view("numpy", stage_id)
                arm = view.create_articulation_view(root)
                if arm.count != 1 or not arm.shared_metatype.fixed_base:
                    raise ValueError("Expected one fixed-base articulation")
                if tuple(arm.shared_metatype.dof_names) != definition.joints:
                    raise ValueError("Isaac joint order differs from robot declaration")
                initial = (
                    definition.default_positions
                    if robot.initial_positions is None
                    else robot.initial_positions
                )
                q = np.asarray(initial, dtype=np.float32).reshape(1, -1)
                if q.shape[1] != len(definition.joints) or not np.isfinite(q).all():
                    raise ValueError("Invalid initial joint positions")
                limits = arm.get_dof_limits()[0]
                if not np.all((q[0] >= limits[:, 0]) & (q[0] <= limits[:, 1])):
                    raise ValueError("Initial joint positions exceed robot limits")
                indices = np.array([0], dtype=np.uint32)
                arm.set_dof_positions(q, indices)
                arm.set_dof_velocities(np.zeros_like(q), indices)
                times.append(0.0)
                positions.append(arm.get_dof_positions()[0].copy())
                for step in range(math.ceil(run.duration / run.time_step)):
                    t = times[-1]
                    if t >= run.duration:
                        break
                    state = np.concatenate(
                        (arm.get_dof_positions()[0], arm.get_dof_velocities()[0])
                    ).astype(float)
                    effort = np.asarray(command(state, t), dtype=np.float32)
                    if (
                        effort.shape != (len(definition.joints),)
                        or not np.isfinite(effort).all()
                    ):
                        raise ValueError("Controller returned invalid effort")
                    arm.set_dof_actuation_forces(effort[None, :], indices)
                    dt = min(run.time_step, run.duration - t)
                    simulation.simulate(dt, t)
                    simulation.fetch_results()
                    measured = arm.get_dof_positions()[0].copy()
                    if not np.isfinite(measured).all():
                        raise RuntimeError("Isaac returned nonfinite joint positions")
                    times.append(min((step + 1) * run.time_step, run.duration))
                    positions.append(measured)
            finally:
                simulation.detach_stage()
                cache.Erase(stage)
        return {
            "times": np.asarray(times),
            "positions": np.asarray(positions),
            "wall_seconds": time.monotonic() - started,
        }
    finally:
        try:
            if trace_path is not None and len(times) >= 1:
                trace_path.parent.mkdir(parents=True, exist_ok=True)
                np.savez(
                    trace_path, times=np.asarray(times), positions=np.asarray(positions)
                )
        finally:
            app.close()
