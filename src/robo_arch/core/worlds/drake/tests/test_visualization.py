"""Inspect native Drake settings and contact diagnostics with a minimal fixture."""

import os
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pydrake")
from pydrake.geometry import (
    AddCompliantHydroelasticProperties,
    AddContactMaterial,
    Box,
    GeometryInstance,
    MakePhongIllustrationProperties,
    ProximityProperties,
    Role,
    Sphere,
)
from pydrake.math import RigidTransform
from pydrake.multibody.plant import ContactModel, CoulombFriction
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import VisualizationConfig

from robo_arch.core.worlds.devices import DeviceDefinitions
from robo_arch.core.worlds.drake.config import DrakePhysics, DrakeVisualization
from robo_arch.core.worlds.drake.scene import DrakeScene, add_ground, add_plant
from robo_arch.core.worlds.drake.visualization import (
    add_visualization,
    create_meshcat,
    hold_live,
    save_recording,
    start_recording,
)


def contact_fixture(config: DrakeVisualization, *, protections: bool = False):
    """A compliant ball on the world's rigid ground; contact properties explicit."""
    builder = DiagramBuilder()
    plant, graph = add_plant(
        builder,
        DrakePhysics(
            time_step=0.002,
            contact_model="hydroelastic",
            discrete_contact_approximation="sap",
        ),
    )
    body = plant.AddRigidBody(
        "ball", SpatialInertia(1.0, [0, 0, 0], UnitInertia.SolidSphere(0.1))
    )
    ball = ProximityProperties()
    AddContactMaterial(1.0, 1e5, CoulombFriction(0.5, 0.5), ball)
    AddCompliantHydroelasticProperties(0.05, 1e6, ball)
    plant.RegisterVisualGeometry(
        body, RigidTransform(), Sphere(0.1), "ball_visual", [0.7, 0.4, 0.2, 1]
    )
    plant.RegisterCollisionGeometry(
        body, RigidTransform(), Sphere(0.1), "ball_collision", ball
    )
    add_ground(plant)
    plant.SetDefaultFloatingBaseBodyPose(body, RigidTransform([0, 0, 0.095]))
    plant.Finalize()
    if protections:
        properties = MakePhongIllustrationProperties([0.1, 0.6, 1, 0.24])
        properties.AddProperty("meshcat", "accepting", "protections")
        overlay = GeometryInstance(
            RigidTransform(), Sphere(0.12), "protection_covering"
        )
        overlay.set_illustration_properties(properties)
        graph.RegisterGeometry(
            plant.get_source_id(), plant.GetBodyFrameIdOrThrow(body.index()), overlay
        )
        obstacle = GeometryInstance(
            RigidTransform([1, 0, 0.2]), Box(0.1, 0.1, 0.1), "obstacle_visual"
        )
        obstacle.set_illustration_properties(
            MakePhongIllustrationProperties([0.9, 0.45, 0.1, 1.0])
        )
        graph.RegisterAnchoredGeometry(plant.get_source_id(), obstacle)
    scene = DrakeScene(
        definitions=DeviceDefinitions(robots={}, sensors={}, objects={}),
        plant=plant,
        scene_graph=graph,
        robots={},
        cameras={},
        wrenches={},
        sensor_instances={},
        controller_models={},
        initial_positions={},
    )
    meshcat = create_meshcat(config)
    if meshcat is not None:
        add_visualization(builder, scene, config, meshcat)
        start_recording(meshcat, config)
    simulator = Simulator(builder.Build())
    simulator.Initialize()
    return simulator, scene, meshcat


def test_native_hydroelastic_layers_and_viewer_independence(tmp_path):
    visualize = os.environ.get("ROBO_ARCH_VISUALIZE") == "1"
    config = DrakeVisualization(
        mode="live_and_record" if visualize else "record",
        open_browser=visualize,
    )
    simulator, scene, meshcat = contact_fixture(config)
    headless, off_scene, off_meshcat = contact_fixture(DrakeVisualization(mode="off"))
    assert off_meshcat is None
    output = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", "recordings"))
    recording = (
        output / "hydroelastic_contact.html" if visualize else tmp_path / "run.html"
    )
    try:
        simulator.AdvanceTo(0.1)
        headless.AdvanceTo(0.1)
        context = scene.plant.GetMyContextFromRoot(simulator.get_context())
        off_context = off_scene.plant.GetMyContextFromRoot(headless.get_context())
        np.testing.assert_allclose(
            scene.plant.GetPositionsAndVelocities(context),
            off_scene.plant.GetPositionsAndVelocities(off_context),
            rtol=0,
            atol=1e-14,
        )
        assert scene.plant.time_step() == 0.002
        assert scene.plant.get_contact_model() == ContactModel.kHydroelastic
        assert scene.plant.get_discrete_contact_approximation().name == "kSap"
        inspector = scene.scene_graph.model_inspector()
        assert inspector.NumGeometriesWithRole(Role.kIllustration) >= 2
        assert inspector.NumGeometriesWithRole(Role.kProximity) == 2
        contacts = scene.plant.get_contact_results_output_port().Eval(context)
        assert contacts.num_hydroelastic_contacts() == 1
        assert contacts.hydroelastic_contact_info(0).contact_surface().num_faces() > 0
        systems = {system.get_name() for system in simulator.get_system().GetSystems()}
        assert "meshcat_visualizer(illustration)" in systems
        assert "meshcat_visualizer(proximity)" in systems
        assert "meshcat_visualizer(inertia)" in systems
        assert "meshcat_contact_visualizer" in systems
        assert not any("mouse" in name.lower() for name in systems)
        assert meshcat.HasPath("/drake/illustration")
        assert meshcat.HasPath("/drake/proximity")
        assert meshcat.HasPath("/drake/inertia")
        assert meshcat.HasPath("/drake/contact_forces/hydroelastic")
    finally:
        save_recording(meshcat, recording)
        print(
            "Inspect same fixture: ROBO_ARCH_VISUALIZE=1 uv run --locked pytest "
            "src/robo_arch/core/worlds/drake/tests/test_visualization.py "
            "-k test_native_hydroelastic -s"
        )
        if visualize:
            print(f"Playback (contact patches are static): {recording.resolve()}")
            if not os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR"):
                hold_live(meshcat)
    assert "<html" in recording.read_text()


def test_visualization_defaults_match_drake():
    config = DrakeVisualization()
    native = VisualizationConfig()
    for name, value in config.model_dump().items():
        if name in {"type", "mode", "open_browser"}:
            continue
        if name in {"default_illustration_color", "default_proximity_color"}:
            np.testing.assert_array_equal(value, getattr(native, name).rgba)
        else:
            assert value == getattr(native, name), name


def test_native_layer_selection():
    config = DrakeVisualization(
        mode="record",
        open_browser=False,
        publish_illustration=False,
        publish_proximity=True,
        enable_alpha_sliders=True,
        initial_proximity_alpha=0.4,
        publish_contacts=False,
        publish_inertia=False,
    )
    simulator, _, meshcat = contact_fixture(config)
    systems = {system.get_name() for system in simulator.get_system().GetSystems()}
    assert "meshcat_visualizer(proximity)" in systems
    assert "meshcat_visualizer(illustration)" not in systems
    assert "meshcat_visualizer(inertia)" not in systems
    assert "meshcat_contact_visualizer" not in systems
    assert meshcat.HasPath("/drake/proximity")
    assert not meshcat.HasPath("/drake/protections")
    assert meshcat.GetSliderValue("proximity α") == pytest.approx(0.4)
    meshcat.StopRecording()


def test_protection_layer_is_exclusive_hidden_and_records_native_motion(tmp_path):
    config = DrakeVisualization(
        mode="record",
        open_browser=False,
        enable_alpha_sliders=True,
        publish_contacts=False,
        publish_inertia=False,
    )
    simulator, scene, meshcat = contact_fixture(config, protections=True)
    layer = "/drake/protections"
    assert meshcat.HasPath(layer + "/ball/protection_covering")
    assert not meshcat.HasPath("/drake/illustration/ball/protection_covering")
    assert meshcat.HasPath("/drake/illustration/ball/ball_visual")
    assert not meshcat.HasPath(layer + "/ball/ball_visual")
    assert meshcat.HasPath("/drake/illustration/obstacle_visual")
    assert not meshcat.HasPath(layer + "/obstacle_visual")
    # Compare native serialized values rather than depend on a msgpack decoder.
    hidden = meshcat._GetPackedProperty(layer, "visible")
    meshcat.SetProperty(layer, "visible", False)
    assert hidden == meshcat._GetPackedProperty(layer, "visible")
    meshcat.SetSliderValue("protections α", 0.4)
    assert meshcat.GetSliderValue("illustration α") == 1.0
    initial_transform = meshcat._GetPackedTransform(layer + "/ball")
    simulator.AdvanceTo(0.04)
    assert meshcat.GetSliderValue("protections α") == 0.4
    assert meshcat._GetPackedTransform(layer + "/ball") != initial_transform
    assert meshcat._GetPackedProperty(layer, "visible") == hidden
    opacity = meshcat._GetPackedProperty(layer, "modulated_opacity")
    assert opacity
    meshcat.SetProperty(layer, "modulated_opacity", 0.4)
    assert meshcat._GetPackedProperty(layer, "modulated_opacity") == opacity
    meshcat.SetProperty(layer, "visible", True)
    assert hidden != meshcat._GetPackedProperty(layer, "visible")
    inspector = scene.scene_graph.model_inspector()
    assert inspector.NumGeometriesWithRole(Role.kProximity) == 2
    for geometry in inspector.GetAllGeometryIds():
        if inspector.GetName(geometry).endswith("covering"):
            assert inspector.GetProximityProperties(geometry) is None
            assert inspector.GetPerceptionProperties(geometry) is None
    recording = tmp_path / "protections.html"
    save_recording(meshcat, recording)
    assert "<html" in recording.read_text()
    print(f"Inspect actual layer motion: python -m webbrowser {recording.as_uri()}")


@pytest.mark.parametrize("approximation", ["sap", "similar", "lagged"])
def test_selected_contact_settings_reach_native_plant(approximation):
    builder = DiagramBuilder()
    plant, _ = add_plant(
        builder,
        DrakePhysics(
            time_step=0.004,
            contact_model="point",
            discrete_contact_approximation=approximation,
            sap_near_rigid_threshold=0.25,
        ),
    )
    plant.Finalize()
    assert plant.time_step() == 0.004
    assert plant.get_sap_near_rigid_threshold() == 0.25
    assert plant.get_contact_model() == ContactModel.kPoint
    assert (
        plant.get_discrete_contact_approximation().name.lower() == f"k{approximation}"
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
