"""Run fixed-base arms with external effort feedback in configured native PhysX."""

import math
import os
import signal
import time
from collections.abc import Callable
from importlib.resources import as_file, files
from pathlib import Path
from tempfile import TemporaryDirectory

import numpy as np

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.objects import load_boxes
from robo_arch.core.worlds.isaac.scene import IsaacScene, build_scene, initialize_scene


def run_scenario(
    run: RunConfiguration,
    *,
    configure: Callable[
        [IsaacScene], dict[str, Callable[[np.ndarray, float], np.ndarray]]
    ],
    trace_path: Path | None = None,
    recording: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict:
    """Step differently sized effort articulations with independent observations.

    Physics can use CPU/GPU; control remains scalar CPU with explicit transfers.
    Sensor bodies are always assembled. Observation selection is independent.
    """
    devices = resolve_devices(run.scene)
    if run.world != "isaac" or not devices.robots:
        raise ValueError("Isaac runner requires fixed-base robots")
    world = run.world_config
    if not isinstance(world, IsaacWorld):
        raise ValueError("Isaac runner requires Isaac world configuration")
    live = world.visualization.mode == "live"
    if recording is not None:
        raise ValueError("Isaac scenario recording is unsupported; use live viewing")
    definitions = load_definitions(run.scene, world.type)
    load_boxes(run.scene, definitions)
    snapshot_path = (
        trace_path.with_suffix(".viewport.png")
        if trace_path is not None and live
        else None
    )
    if live and not os.environ.get("DISPLAY"):
        raise RuntimeError(
            "Isaac live viewing requires a desktop DISPLAY (X11/XWayland)"
        )
    if snapshot_path is not None and not live:
        raise ValueError("A viewport snapshot requires Isaac live viewing")
    if live:
        # Match OpenGL to Kit's NVIDIA device on hybrid Intel/NVIDIA desktops.
        os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
        os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")
    from isaacsim import SimulationApp

    started = time.monotonic()
    experience = files(__package__).joinpath("viewport.kit" if live else "physics.kit")
    interrupt_handler = signal.getsignal(signal.SIGINT)
    with as_file(experience) as path:
        app = SimulationApp(
            {
                "headless": not live,
                "width": world.visualization.width,
                "height": world.visualization.height,
                "window_width": world.visualization.width,
                "window_height": world.visualization.height,
                "minimal_shading_mode": 3,
                "anti_aliasing": 0,
                "denoiser": False,
                "physics_gpu": 0,
                "multi_gpu": False,
                "renderer": "MinimalRendering",
                "create_new_stage": False,
                "fast_shutdown": False,
            },
            experience=str(path),
        )
    # Let Ctrl-C unwind our finally blocks instead of Kit's immediate shutdown.
    signal.signal(signal.SIGINT, interrupt_handler)
    try:
        result = _simulate(
            app,
            run,
            configure,
            trace_path=trace_path,
            snapshot_path=snapshot_path,
            keep_viewer_open=keep_viewer_open,
        )
        result["wall_seconds"] = time.monotonic() - started
        result["viewport_image"] = (
            str(snapshot_path.resolve()) if snapshot_path else None
        )
        return result
    finally:
        # A second Ctrl-C during native extension teardown can interrupt cleanup.
        cleanup_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            app.close()
        finally:
            signal.signal(signal.SIGINT, cleanup_handler)


def _simulate(app, run, configure, *, trace_path, snapshot_path, keep_viewer_open):
    """Release native stage/tensor/viewer wrappers before shutting down Kit."""
    devices = resolve_devices(run.scene)
    world = run.world_config
    live = world.visualization.mode == "live"
    times: list[float] = []
    samples: dict[str, list[np.ndarray]] = {}
    import omni.physics.tensors as tensors
    from omni.physx import get_physx_simulation_interface
    from pxr import UsdUtils

    with TemporaryDirectory(prefix="robo_arch_isaac_") as directory:
        scene = build_scene(run.scene, world, directory=Path(directory))
        stage, physics = scene.stage, scene.physics
        commands = configure(scene)
        if set(commands) != {robot.name for robot in devices.robots}:
            raise ValueError("Commands must name exactly the configured robots")
        cache = UsdUtils.StageCache.Get()
        stage_id = cache.Insert(stage).ToLongInt()
        simulation = get_physx_simulation_interface()
        viewer = None
        try:
            if live:
                from robo_arch.core.worlds.isaac.visualization import Viewer

                viewer = Viewer(app, stage, world.visualization)
            simulation.attach_stage(stage_id)
            simulation.simulate(run.time_step, 0.0)
            simulation.fetch_results()
            view = tensors.create_simulation_view("numpy", stage_id)
            arms, sensors, efforts = initialize_scene(scene, view)
            indices = np.array([0], dtype=np.uint32)

            def sample(t):
                values = {}
                for name, arm in arms.items():
                    values[name + "/q"] = arm.get_dof_positions()[0]
                    values[name + "/v"] = arm.get_dof_velocities()[0]
                    values[name + "/effort"] = efforts[name]
                for name, observe in sensors.items():
                    # Solver reactions are meaningful only after a step from
                    # the selected initial state, not the initialization step.
                    values[name + "/wrench"] = (
                        observe() if t > 0 else np.full(6, np.nan)
                    )
                times.append(t)
                for name, value in values.items():
                    samples.setdefault(name, []).append(
                        np.array(value, dtype=float, copy=True)
                    )

            sample(0.0)
            if viewer is not None:
                viewer.wall_start = time.monotonic()
            for step in range(math.ceil(run.duration / run.time_step)):
                t = times[-1]
                for name, arm in arms.items():
                    state = np.concatenate(
                        (arm.get_dof_positions()[0], arm.get_dof_velocities()[0])
                    ).astype(float)
                    effort = np.asarray(commands[name](state, t), dtype=np.float32)
                    if (
                        effort.shape != efforts[name].shape
                        or not np.isfinite(effort).all()
                    ):
                        raise ValueError(
                            f"Controller returned invalid effort for {name}"
                        )
                    efforts[name] = effort
                    arm.set_dof_actuation_forces(effort[None, :], indices)
                dt = min(run.time_step, run.duration - t)
                simulation.simulate(dt, t)
                simulation.fetch_results()
                sample(min((step + 1) * run.time_step, run.duration))
                if viewer is not None:
                    viewer.update(times[-1], world.target_realtime_rate)
                for name in arms:
                    if not np.isfinite(samples[name + "/q"][-1]).all():
                        raise RuntimeError(f"Isaac returned nonfinite state for {name}")
            if viewer is not None:
                if snapshot_path is not None:
                    viewer.capture(snapshot_path)
                if keep_viewer_open:
                    viewer.hold()
        finally:
            try:
                if trace_path is not None and times:
                    trace_path.parent.mkdir(parents=True, exist_ok=True)
                    np.savez(trace_path, times=times, **samples)
            finally:
                try:
                    simulation.detach_stage()
                finally:
                    try:
                        if viewer is not None:
                            viewer.close()
                    finally:
                        cache.Erase(stage)
    return {
        "trace": {
            "times": np.asarray(times),
            **{name: np.asarray(values) for name, values in samples.items()},
        },
        "physics_settings": {
            "solver": physics.GetSolverTypeAttr().Get(),
            "gpu_dynamics": physics.GetEnableGPUDynamicsAttr().Get(),
            "broadphase": physics.GetBroadphaseTypeAttr().Get(),
        },
    }
