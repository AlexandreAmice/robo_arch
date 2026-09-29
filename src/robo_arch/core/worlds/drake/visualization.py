"""Render recorded joint positions without rerunning the source world's physics."""

from dataclasses import replace
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat, MeshcatParams, MeshcatVisualizer
from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.config.loading import RunConfiguration
from robo_arch.core.worlds.drake.scene import build_scene
from robo_arch.core.worlds.registry import Registry


def replay_positions(
    run: RunConfiguration,
    registry: Registry,
    trace_path: Path,
    recording: Path,
) -> None:
    """Replay a single arm's measured trace with Drake model geometry in Meshcat."""
    with np.load(trace_path) as trace:
        times, positions = trace["times"], trace["positions"]
    if len(times) == 0:
        return
    builder = DiagramBuilder()
    scene = build_scene(builder, replace(run, world="drake", sensors=()), registry)
    meshcat = Meshcat(MeshcatParams(host="localhost"))
    meshcat.SetCameraPose([0.9, -0.9, 0.8], [0.25, 0.0, 0.3])
    visualizer = MeshcatVisualizer.AddToBuilder(builder, scene.scene_graph, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = scene.plant.GetMyMutableContextFromRoot(context)
    visualizer_context = visualizer.GetMyContextFromRoot(context)
    meshcat.StartRecording()
    # Render at roughly 30 Hz, always retaining the final recorded state.
    stride = max(1, int(1 / (30 * run.time_step)))
    indices = sorted(set(range(0, len(times), stride)) | {len(times) - 1})
    for index in indices:
        context.SetTime(times[index])
        scene.plant.SetPositions(
            plant_context, scene.robots[run.robots[0].name], positions[index]
        )
        visualizer.ForcedPublish(visualizer_context)
    meshcat.StopRecording()
    meshcat.PublishRecording()
    recording.parent.mkdir(parents=True, exist_ok=True)
    recording.write_text(meshcat.StaticHtml(), encoding="utf-8")
