"""Run shared effort controllers in Isaac Lab with independent environment reset."""

import gc
import math
import os
import signal
import sys
import time
import traceback
from collections.abc import Callable, Sequence
from importlib.resources import as_file, files
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import Any

import numpy as np

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.isaac.backend import validate_observations
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.objects import load_boxes
from robo_arch.core.worlds.isaac.scene import IsaacScene, build_scene, initialize_scene


def run_scenario(
    run: RunConfiguration,
    *,
    configure: Callable[
        [IsaacScene], dict[str, Callable[[np.ndarray, float], np.ndarray]]
    ]
    | None = None,
    rollout: Callable[[IsaacScene, Any], dict] | None = None,
    trace_path: Path | None = None,
    recording: Path | None = None,
    keep_viewer_open: bool = False,
    after_step: Callable[["Execution"], None] | None = None,
) -> dict:
    """Step differently sized effort articulations with independent observations.

    configure selects scalar CPU callbacks; rollout selects a scenario-owned
    batched loop over the same initialized scene and viewer. Supply exactly one.
    Sensor bodies are always assembled. Observation selection is independent.
    """
    if (configure is None) == (rollout is None):
        raise ValueError("Provide exactly one of configure or rollout")
    if rollout is not None and after_step is not None:
        raise ValueError("after_step is only supported with scalar configure callbacks")
    devices = resolve_devices(run.scene)
    if run.world != "isaac" or not devices.robots:
        raise ValueError("Isaac runner requires fixed-base robots")
    world = run.world_config
    if not isinstance(world, IsaacWorld):
        raise ValueError("Isaac runner requires Isaac world configuration")
    validate_observations(
        world.physics,
        tuple(sensor.name for sensor in devices.sensors)
        if run.scene.sensors_enabled
        else (),
    )
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
    from isaaclab.app import AppLauncher

    started = time.monotonic()
    experience = files(__package__).joinpath("viewport.kit" if live else "physics.kit")
    interrupt_handler = signal.getsignal(signal.SIGINT)
    with as_file(experience) as path:
        launcher = AppLauncher(
            {
                "visualizer": ["kit"] if live else ["none"],
                "device": world.physics.device,
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
    app = launcher.app
    # Storm inspection below owns rendering; do not also construct Lab's RTX
    # KitVisualizer merely because AppLauncher opened a desktop window.
    AppLauncher.sync_visualizer_cli_settings_to_carb(
        {"visualizer": [], "visualizer_explicit": True, "visualizer_disable_all": True}
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
            after_step=after_step,
            rollout=rollout,
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
            # Lab 3.0.0rc1 retains native handles after clear_instance().
            # Release them before Kit unloads their plugins.
            if world.physics.backend == "physx":
                import isaaclab_physx
                from isaaclab_physx.physics import PhysxManager

                isaaclab_physx._SIMULATION_MANAGER_ENABLE_HOOK = None
                PhysxManager._timeline = None
                PhysxManager._event_bus = None
                PhysxManager._message_bus = None
                PhysxManager._scene_data_backend = None
            launcher._hide_play_button_callback = None
            launcher._unhide_play_button_callback = None
            # An exception keeps completed simulation frames (and their native
            # views) alive. Preserve the traceback locations, but release locals
            # before unloading Kit. Partial state is retained in the NPZ trace.
            if error := sys.exception():
                traceback.clear_frames(error.__traceback__)
            gc.collect()
            app.close()
        finally:
            signal.signal(signal.SIGINT, cleanup_handler)


class Execution:
    """Own per-environment autonomy and episode clocks for one Lab scene.

    Reset constructs fresh controller callbacks only for selected environments,
    so private model contexts and future controller memory cannot leak between
    episodes. Physics and sensor reset stay with the native Lab scene.
    """

    def __init__(
        self,
        scene: IsaacScene,
        configure: Callable[
            [IsaacScene], dict[str, Callable[[np.ndarray, float], np.ndarray]]
        ],
    ) -> None:
        self.scene = scene
        self.configure = configure
        self.initial = initialize_scene(scene)
        self.commands = [{} for _ in range(scene.world.num_envs)]
        self.episode_times = np.zeros(scene.world.num_envs)
        self.time = 0.0
        self.reset_events = []
        self.efforts = {
            name: np.zeros((scene.world.num_envs, len(q)))
            for name, q in self.initial.items()
        }
        self.reset(range(scene.world.num_envs))
        self.reset_events.clear()

    def reset(self, env_ids: Sequence[int]) -> None:
        """Restore selected environments; empty selection is a no-op.

        IDs must be distinct integers in range. A fresh controller construction
        failure propagates before changing physical state.
        """
        ids = list(env_ids)
        if any(type(i) is not int or not 0 <= i < len(self.commands) for i in ids):
            raise ValueError("Reset IDs must be integer environment indices in range")
        if len(ids) != len(set(ids)):
            raise ValueError("Reset IDs must be distinct")
        if not ids:
            return
        fresh = [self.configure(self.scene) for _ in ids]
        if any(set(commands) != set(self.initial) for commands in fresh):
            raise ValueError("Commands must name exactly the configured robots")
        import torch

        device = self.scene.world.physics.device
        indices = torch.tensor(ids, dtype=torch.int32, device=device)
        self.scene.native.reset(ids)
        for name, arm in self.scene.native.articulations.items():
            q = torch.as_tensor(self.initial[name], device=device).repeat(len(ids), 1)
            arm.write_joint_state_to_sim_index(
                position=q, velocity=torch.zeros_like(q), env_ids=indices
            )
            arm.actuators.target_command.set_effort_index(
                value=torch.zeros_like(q), env_ids=indices
            )
            self.efforts[name][ids] = 0
        for env, commands in zip(ids, fresh, strict=True):
            self.commands[env] = commands
        self.episode_times[ids] = 0
        self.reset_events.append({"time_seconds": self.time, "env_ids": ids})

    def step(self, dt: float) -> None:
        """Evaluate the shared CPU controllers and advance Lab once."""
        import torch

        for name, arm in self.scene.native.articulations.items():
            q = arm.data.joint_pos.torch.cpu().numpy()
            v = arm.data.joint_vel.torch.cpu().numpy()
            for env, commands in enumerate(self.commands):
                effort = np.asarray(
                    commands[name](np.r_[q[env], v[env]], self.episode_times[env]),
                    dtype=float,
                )
                if (
                    effort.shape != self.initial[name].shape
                    or not np.isfinite(effort).all()
                ):
                    raise ValueError(
                        f"Controller returned invalid effort for env {env}/{name}"
                    )
                self.efforts[name][env] = effort
            arm.actuators.target_command.set_effort_index(
                value=torch.as_tensor(
                    self.efforts[name],
                    dtype=torch.float32,
                    device=self.scene.world.physics.device,
                )
            )
        self.scene.native.write_data_to_sim()
        # Lab's PhysX manager reads cfg.dt when stepping; retain the final partial
        # step so reported time never exceeds the requested run duration.
        self.scene.simulation.cfg.dt = dt
        self.scene.simulation.step(render=False)
        self.scene.native.update(dt)
        self.time += dt
        self.episode_times += dt

    def sample(self) -> dict[str, np.ndarray]:
        """Copy samples to CPU; one environment retains the original trace keys."""
        values = {}
        for name, arm in self.scene.native.articulations.items():
            values[name + "/q"] = arm.data.joint_pos.torch.cpu().numpy().copy()
            values[name + "/v"] = arm.data.joint_vel.torch.cpu().numpy().copy()
            values[name + "/effort"] = self.efforts[name].copy()
            if not np.isfinite(values[name + "/q"]).all():
                raise RuntimeError(f"Isaac Lab returned nonfinite state for {name}")
        for name, observe in self.scene.observations.items():
            values[name + "/wrench"] = observe()
            values[name + "/wrench"][self.episode_times == 0] = np.nan
        if len(self.commands) == 1:
            return {name: value[0] for name, value in values.items()}
        return {
            **{
                f"env_{env}/{name}": value[env]
                for name, value in values.items()
                for env in range(len(self.commands))
            },
            **{
                f"env_{env}/episode_time": np.array(t)
                for env, t in enumerate(self.episode_times)
            },
        }


def _simulate(
    app: Any,
    run: RunConfiguration,
    configure: Callable[
        [IsaacScene], dict[str, Callable[[np.ndarray, float], np.ndarray]]
    ],
    *,
    trace_path: Path | None,
    snapshot_path: Path | None,
    keep_viewer_open: bool,
    after_step: Callable[[Execution], None] | None,
    rollout: Callable[[IsaacScene, Any], dict] | None,
) -> dict:
    """Release Lab scene/physics and viewer resources before Kit shutdown."""
    from isaaclab.sim import SimulationContext
    from pxr import PhysxSchema

    world = run.world_config
    times = []
    samples = {}
    viewer = None
    with TemporaryDirectory(prefix="robo_arch_isaac_") as directory:
        try:
            scene = build_scene(run.scene, world, directory=Path(directory))
            if world.visualization.mode == "live":
                from robo_arch.core.worlds.isaac.visualization import Viewer

                viewer = Viewer(app, scene, world.visualization)
            scene.simulation.reset()
            # Lab steps explicitly; Kit updates serve the viewport and cleanup.
            scene.simulation.set_setting("/app/player/playSimulations", False)
            if rollout is not None:
                result = rollout(scene, viewer)
            else:
                execution = Execution(scene, configure)

                def sample():
                    values = execution.sample()
                    times.append(execution.time)
                    for name, value in values.items():
                        samples.setdefault(name, []).append(value)

                sample()
                if viewer is not None:
                    viewer.wall_start = time.monotonic()
                for _step in range(math.ceil(run.duration / run.time_step)):
                    execution.step(min(run.time_step, run.duration - execution.time))
                    if after_step is not None:
                        after_step(execution)
                    sample()
                    if viewer is not None:
                        viewer.update(execution.time, world.target_realtime_rate)
                result = {
                    "trace": {
                        "times": np.asarray(times),
                        **{k: np.asarray(v) for k, v in samples.items()},
                    },
                    "reset_events": execution.reset_events,
                }
            if viewer is not None:
                if snapshot_path is not None:
                    viewer.capture(snapshot_path)
                if keep_viewer_open:
                    viewer.hold()
            settings = world.physics.model_dump(mode="json")
            if world.physics.backend == "physx":
                physics = PhysxSchema.PhysxSceneAPI.Get(
                    scene.stage, scene.simulation.cfg.physics_prim_path
                )
                settings.update(
                    solver=physics.GetSolverTypeAttr().Get(),
                    gpu_dynamics=physics.GetEnableGPUDynamicsAttr().Get(),
                    broadphase=physics.GetBroadphaseTypeAttr().Get(),
                )
            result.update(
                physics_settings=settings,
                framework="isaaclab",
                physics_backend=(
                    "isaacsim_physx"
                    if world.physics.backend == "physx"
                    else "newton_mujoco_warp"
                ),
                num_envs=world.num_envs,
            )
            return result

        finally:
            try:
                if trace_path is not None and times:
                    trace_path.parent.mkdir(parents=True, exist_ok=True)
                    np.savez(trace_path, times=times, **samples)
            finally:
                try:
                    if simulation := SimulationContext.instance():
                        simulation.stop()
                finally:
                    try:
                        if viewer is not None:
                            viewer.pause_rendering()
                    finally:
                        try:
                            SimulationContext.clear_instance()
                        finally:
                            # Lab pumps Kit while closing. Fence rendering again
                            # after that work, before releasing viewport handles.
                            if viewer is not None:
                                viewer.close(close_stage=False)
