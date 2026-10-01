"""Drake's native observational viewer and explicit geometry-only trace replay."""

import time
import webbrowser
from dataclasses import replace
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat, MeshcatParams, Rgba
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.lcm import ApplyLcmBusConfig
from pydrake.visualization import ApplyVisualizationConfig, VisualizationConfig

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import DeviceDefinitions
from robo_arch.core.worlds.drake.config import DrakeVisualization, DrakeWorld
from robo_arch.core.worlds.drake.scene import DrakeScene, build_scene


def create_meshcat(config: DrakeVisualization) -> Meshcat | None:
    """Create a localhost viewer unless disabled; caller owns its lifetime.

    Record mode also needs Meshcat's native server to generate standalone HTML,
    but does not require a browser. Dropping the last reference stops the server.
    """
    if config.mode == "off":
        return None
    meshcat = Meshcat(MeshcatParams(host="localhost"))
    meshcat.SetCameraPose([0.9, -0.9, 0.8], [0.25, 0.0, 0.3])
    if config.open_browser and config.mode in {"live", "live_and_record"}:
        webbrowser.open(meshcat.web_url())
    return meshcat


def add_visualization(
    builder: DiagramBuilder,
    scene: DrakeScene,
    config: DrakeVisualization,
    meshcat: Meshcat,
) -> None:
    """Wire standard illustration, proximity, inertia and contact layers.

    Hydroelastic meshes use the asset's proximity properties. This never adds
    contact geometry or changes materials. Mouse-applied forces are disabled.
    No external LCM transport is needed for this Meshcat-only profile.
    """
    if config.mode == "off":
        return
    ApplyVisualizationConfig(
        VisualizationConfig(
            publish_period=config.publish_period,
            publish_illustration=config.publish_illustration,
            default_illustration_color=Rgba(*config.default_illustration_color),
            publish_proximity=config.publish_proximity,
            default_proximity_color=Rgba(*config.default_proximity_color),
            initial_proximity_alpha=config.initial_proximity_alpha,
            publish_contacts=config.publish_contacts,
            publish_inertia=config.publish_inertia,
            delete_on_initialization_event=config.delete_on_initialization_event,
            enable_alpha_sliders=config.enable_alpha_sliders,
            enable_meshcat_creation=False,
            mouse_interaction_stiffness=None,
        ),
        builder=builder,
        plant=scene.plant,
        scene_graph=scene.scene_graph,
        meshcat=meshcat,
        lcm_buses=ApplyLcmBusConfig({"default": None}, builder),
    )


def start_recording(meshcat: Meshcat, config: DrakeVisualization) -> None:
    """Begin native recording at the configured display cadence when requested."""
    if config.mode in {"record", "live_and_record"}:
        meshcat.StartRecording(frames_per_second=1.0 / config.publish_period)


def save_recording(meshcat: Meshcat, recording: Path) -> None:
    """Retain native playback, including partial runs when called in finally.

    Meshcat records geometry transforms and force arrows, but does not faithfully
    replay changing hydroelastic contact patches or pressure fields. Inspect
    those live. The HTML can contain the last published patch as a static mesh.
    """
    meshcat.StopRecording()
    meshcat.PublishRecording()
    recording.parent.mkdir(parents=True, exist_ok=True)
    recording.write_text(meshcat.StaticHtml(), encoding="utf-8")


def hold_live(meshcat: Meshcat) -> None:
    """Keep the final scene available until its stop button or Ctrl-C is used."""
    button = "Close inspection"
    meshcat.AddButton(button)
    try:
        while meshcat.GetButtonClicks(button) == 0:
            time.sleep(0.1)
    finally:
        meshcat.DeleteButton(button)


def replay_positions(
    run: RunConfiguration,
    definitions: DeviceDefinitions,
    trace_path: Path,
    recording: Path,
) -> None:
    """Replay one arm's positions as secondary Drake geometry inspection.

    This is not the source world's native viewer. Positions cannot reconstruct
    its contact forces, hydroelastic pressure, sensor observations or velocities;
    contact publication is therefore disabled. No physics is advanced.
    """
    with np.load(trace_path) as trace:
        times, positions = trace["times"], trace["positions"]
    if len(times) == 0:
        return
    devices = resolve_devices(run.scene)
    config = DrakeVisualization(mode="record", publish_contacts=False)
    builder = DiagramBuilder()
    scene = build_scene(
        replace(run.scene, sensors_enabled=False),
        DrakeWorld(visualization=config),
        builder=builder,
    )
    meshcat = create_meshcat(config)
    add_visualization(builder, scene, config, meshcat)
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = scene.plant.GetMyMutableContextFromRoot(context)
    start_recording(meshcat, config)
    # Render at roughly 30 Hz, always retaining the final recorded state.
    stride = max(1, int(1 / (30 * run.time_step)))
    indices = sorted(set(range(0, len(times), stride)) | {len(times) - 1})
    try:
        for index in indices:
            context.SetTime(times[index])
            scene.plant.SetPositions(
                plant_context, scene.robots[devices.robots[0].name], positions[index]
            )
            diagram.ForcedPublish(context)
    finally:
        save_recording(meshcat, recording)
