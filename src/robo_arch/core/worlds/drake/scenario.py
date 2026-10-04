"""Construct and execute configured scenarios in the native Drake runtime."""

import sys
import webbrowser
from collections.abc import Callable, Mapping
from pathlib import Path
from time import perf_counter

import numpy as np
from pydrake.geometry import Meshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, OutputPort
from pydrake.systems.primitives import LogVectorOutput, VectorLogSink

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.drake.scene import DrakeScene, build_scene
from robo_arch.core.worlds.drake.visualization import (
    add_visualization,
    create_meshcat,
    hold_live,
    save_recording,
    start_recording,
)

Configure = Callable[[DiagramBuilder, DrakeScene], Mapping[str, OutputPort] | None]


def build_simulation(
    run: RunConfiguration,
    *,
    configure: Configure,
    meshcat: Meshcat | None = None,
    initialize: bool = True,
) -> tuple[Simulator, DrakeScene]:
    """Build physics and autonomy; initialize unless the caller owns that step.

    Delayed initialization lets runners retain logs if initialization raises.
    """
    if not isinstance(run.world_config, DrakeWorld):
        raise ValueError("Drake scenario execution requires DrakeWorld")
    builder = DiagramBuilder()
    scene = build_scene(run.scene, run.world_config, builder=builder)
    configure(builder, scene)
    if meshcat is not None:
        add_visualization(builder, scene, run.world_config.visualization, meshcat)
    simulator = Simulator(builder.Build())
    simulator.set_target_realtime_rate(run.world_config.target_realtime_rate)
    context = scene.plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    for name, positions in scene.initial_positions.items():
        scene.plant.SetPositions(context, scene.robots[name], positions)
    if initialize:
        simulator.Initialize()
    return simulator, scene


def run_scenario(
    run: RunConfiguration,
    *,
    configure: Configure,
    recording: Path | None = None,
    trace_path: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict:
    """Run physics and export native logs, including optional scenario signals.

    ``configure`` may return named vector output ports. Each additional channel
    has its own ``<name>/times`` array so partial failure logs remain interpretable.
    Existing robot channels retain their common ``times`` array.
    """
    if not isinstance(run.world_config, DrakeWorld):
        raise ValueError("Drake scenario execution requires DrakeWorld")
    if recording is not None:
        if run.world_config.visualization.mode not in {"record", "live_and_record"}:
            raise ValueError("A recording path requires a recording visualization mode")
        recording = recording.resolve()
        recording.parent.mkdir(parents=True, exist_ok=True)
    visual = run.world_config.visualization
    meshcat = create_meshcat(visual)
    logs: dict[str, VectorLogSink] = {}
    additional_logs: dict[str, VectorLogSink] = {}
    simulator = None

    def configure_and_log(builder: DiagramBuilder, scene: DrakeScene) -> None:
        additional = configure(builder, scene) or {}
        for name, instance in scene.robots.items():
            logs[name + "/state"] = LogVectorOutput(
                scene.plant.get_state_output_port(instance), builder
            )
            logs[name + "/effort"] = LogVectorOutput(
                scene.plant.get_net_actuation_output_port(instance), builder
            )
        for name, sensor in scene.wrenches.items():
            logs[name + "/wrench"] = LogVectorOutput(sensor.get_output_port(), builder)
        reserved = {"times", *logs}
        for name in scene.robots:
            reserved.update((name + "/q", name + "/v"))
        reserved.update(name + "/times" for name in tuple(reserved))
        channel_names = {key for name in additional for key in (name, name + "/times")}
        if len(channel_names) != 2 * len(additional) or channel_names & reserved:
            raise ValueError("Scenario log names overlap each other or world channels")
        for name, port in additional.items():
            if not name:
                raise ValueError("Scenario log names must be nonempty")
            additional_logs[name] = LogVectorOutput(port, builder)

    def export_logs() -> dict[str, np.ndarray]:
        context = simulator.get_context()
        data = {}
        for name, sink in logs.items():
            log = sink.FindLog(context)
            times = log.sample_times().copy()
            data.setdefault("times", times)
            # A publish failure can interrupt a group of native loggers. Keep
            # their partial samples without assigning another logger's times.
            if not np.array_equal(data["times"], times):
                if name.endswith("/state"):
                    robot = name.removesuffix("/state")
                    data[robot + "/q/times"] = times
                    data[robot + "/v/times"] = times
                else:
                    data[name + "/times"] = times
            values = log.data().T.copy()
            if name.endswith("/state"):
                robot = name.removesuffix("/state")
                nq = scene.plant.num_positions(scene.robots[robot])
                data[robot + "/q"] = values[:, :nq]
                data[robot + "/v"] = values[:, nq:]
            else:
                if name.endswith("/wrench"):
                    # No solver reaction exists before the first physics step.
                    values[times == 0] = np.nan
                data[name] = values
        for name, sink in additional_logs.items():
            log = sink.FindLog(context)
            data[name + "/times"] = log.sample_times().copy()
            data[name] = log.data().T.copy()
        return data

    if meshcat is not None:
        start_recording(meshcat, visual)
    try:
        simulator, scene = build_simulation(
            run, configure=configure_and_log, meshcat=meshcat, initialize=False
        )
        simulator.Initialize()

        def depth_counts() -> dict[str, int]:
            return {
                name: int(
                    np.isfinite(
                        camera.depth_image_32F_output_port()
                        .Eval(camera.GetMyContextFromRoot(simulator.get_context()))
                        .data
                    ).sum()
                )
                for name, camera in scene.cameras.items()
            }

        initial_depth = depth_counts()
        started = perf_counter()
        simulator.AdvanceTo(run.duration)
        simulation_wall_seconds = perf_counter() - started
        if recording is not None:
            from pydrake.systems.sensors import ImageIo

            for name, camera in scene.cameras.items():
                image = camera.color_image_output_port().Eval(
                    camera.GetMyContextFromRoot(simulator.get_context())
                )
                destination = recording.with_name(
                    f"{recording.stem}_{name.replace('/', '__')}.png"
                )
                ImageIo().Save(image, destination)
        return {
            "trace": export_logs(),
            "simulation_wall_seconds": simulation_wall_seconds,
            "realtime_rate": run.duration / simulation_wall_seconds,
            "initial_finite_depth_pixels": initial_depth,
            "final_finite_depth_pixels": depth_counts(),
        }
    finally:
        if trace_path is not None and simulator is not None:
            trace_path.parent.mkdir(parents=True, exist_ok=True)
            np.savez(trace_path, **export_logs())
        if meshcat is not None:
            if recording is not None:
                save_recording(meshcat, recording)
                print(f"Scene playback: {recording.as_uri()}", file=sys.stderr)
                if visual.mode == "record" and visual.open_browser:
                    webbrowser.open(recording.as_uri())
            if keep_viewer_open and visual.mode in {"live", "live_and_record"}:
                hold_live(meshcat)
