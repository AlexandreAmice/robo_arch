"""Construct and execute configured scenarios in the native Drake runtime."""

import math
import sys
import webbrowser
from collections.abc import Callable
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

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
from robo_arch.core.worlds.traces import Trace


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
    """Own native simulation and viewing; return traces for scenario evaluation."""
    if not isinstance(run.world_config, DrakeWorld):
        raise ValueError("Drake scenario execution requires DrakeWorld")
    if recording is not None:
        if run.world_config.visualization.mode not in {"record", "live_and_record"}:
            raise ValueError("A recording path requires a recording visualization mode")
        recording = recording.resolve()
        recording.parent.mkdir(parents=True, exist_ok=True)
    visual = run.world_config.visualization
    meshcat = create_meshcat(visual)
    trace = Trace()
    if meshcat is not None:
        start_recording(meshcat, visual)
    try:
        simulator, scene = build_simulation(run, configure=configure, meshcat=meshcat)

        def sample():
            context = scene.plant.GetMyContextFromRoot(simulator.get_context())
            values = {}
            for name, instance in scene.robots.items():
                values[name + "/q"] = scene.plant.GetPositions(context, instance)
                values[name + "/v"] = scene.plant.GetVelocities(context, instance)
                values[name + "/effort"] = scene.plant.get_actuation_input_port(
                    instance
                ).Eval(context)
            for name, sensor in scene.wrenches.items():
                sensor_context = sensor.GetMyContextFromRoot(simulator.get_context())
                values[name + "/wrench"] = (
                    sensor.get_output_port().Eval(sensor_context)
                    if context.get_time() > 0
                    else np.full(6, np.nan)
                )
            trace.append(context.get_time(), values)

        def depth_counts():
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
        sample()
        for step in range(math.ceil(run.duration / run.time_step)):
            simulator.AdvanceTo(min((step + 1) * run.time_step, run.duration))
            sample()
        if meshcat is not None:
            simulator.get_system().ForcedPublish(simulator.get_context())
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
            "trace": trace,
            "initial_finite_depth_pixels": initial_depth,
            "final_finite_depth_pixels": depth_counts(),
        }
    finally:
        if trace_path is not None and trace.times:
            trace.save(trace_path)
        if meshcat is not None:
            if recording is not None:
                save_recording(meshcat, recording)
                print(f"Scene playback: {recording.as_uri()}", file=sys.stderr)
                if visual.mode == "record" and visual.open_browser:
                    webbrowser.open(recording.as_uri())
            if keep_viewer_open and visual.mode in {"live", "live_and_record"}:
                hold_live(meshcat)
