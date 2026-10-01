"""Construct and execute configured scenarios in the native Drake runtime."""

import sys
import webbrowser
from collections.abc import Callable
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
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


def build_simulation(
    run: RunConfiguration,
    *,
    configure: Callable[[DiagramBuilder, DrakeScene], None],
    meshcat: Meshcat | None = None,
) -> tuple[Simulator, DrakeScene]:
    """Build physics, invoke scenario wiring, then initialize the native simulator."""
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
    simulator.Initialize()
    return simulator, scene


def run_scenario(
    run: RunConfiguration,
    *,
    configure: Callable[[DiagramBuilder, DrakeScene], None],
    recording: Path | None = None,
    trace_path: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict:
    """Run the native simulator; export native signal logs for evaluation."""
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
    simulator = None

    def configure_and_log(builder: DiagramBuilder, scene: DrakeScene) -> None:
        configure(builder, scene)
        for name, instance in scene.robots.items():
            logs[name + "/state"] = LogVectorOutput(
                scene.plant.get_state_output_port(instance), builder
            )
            logs[name + "/effort"] = LogVectorOutput(
                scene.plant.get_net_actuation_output_port(instance), builder
            )
        for name, sensor in scene.wrenches.items():
            logs[name + "/wrench"] = LogVectorOutput(sensor.get_output_port(), builder)

    def export_logs() -> dict[str, np.ndarray]:
        context = simulator.get_context()
        data = {}
        for name, sink in logs.items():
            log = sink.FindLog(context)
            data["times"] = log.sample_times().copy()
            values = log.data().T.copy()
            if name.endswith("/state"):
                robot = name.removesuffix("/state")
                nq = scene.plant.num_positions(scene.robots[robot])
                data[robot + "/q"] = values[:, :nq]
                data[robot + "/v"] = values[:, nq:]
            else:
                if name.endswith("/wrench"):
                    # No solver reaction exists before the first physics step.
                    values[data["times"] == 0] = np.nan
                data[name] = values
        return data

    if meshcat is not None:
        start_recording(meshcat, visual)
    try:
        simulator, scene = build_simulation(
            run, configure=configure_and_log, meshcat=meshcat
        )

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
        simulator.AdvanceTo(run.duration)
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
