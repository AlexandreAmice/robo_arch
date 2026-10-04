"""World-owned assembly and execution without a concrete task implementation."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pydrake")
from pydrake.geometry import HalfSpace
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from robo_arch.core.config.declarations import (
    AutonomySelection,
    Pose,
    RobotInstance,
    RobotSystem,
    RunConfiguration,
    SceneConfiguration,
    TaskSelection,
)
from robo_arch.core.config.loading import load_robot
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.drake.scenario import run_scenario
from robo_arch.core.worlds.drake.scene import build_scene


@pytest.mark.parametrize("ground", [True, False])
def test_ground_is_physical_and_visible_independently_of_viewer(ground):
    scene_config = SceneConfiguration(
        robot_system=RobotSystem(source=Path("system.yaml"), name="", pose=Pose()),
        sensors_enabled=False,
        objects=(),
    )
    builder = DiagramBuilder()
    scene = build_scene(scene_config, DrakeWorld(ground=ground), builder=builder)
    inspector = scene.scene_graph.model_inspector()
    collision = []
    visual = []
    for geometry in inspector.GetAllGeometryIds():
        if inspector.GetProximityProperties(geometry) is not None:
            collision.append(geometry)
        if inspector.GetIllustrationProperties(geometry) is not None:
            visual.append(geometry)
            assert inspector.GetPerceptionProperties(geometry) is not None
    assert len(collision) == len(visual) == int(ground)
    if ground:
        assert isinstance(inspector.GetShape(collision[0]), HalfSpace)
        np.testing.assert_allclose(
            inspector.GetPoseInFrame(collision[0]).translation(), 0
        )
        shape = inspector.GetShape(visual[0])
        top = inspector.GetPoseInFrame(visual[0]).translation()[2] + shape.height() / 2
        assert top == pytest.approx(0.0)


@pytest.mark.parametrize("initialization_failure", [None, "before", "after"])
def test_scene_and_scenario_have_independent_entry_points(
    tmp_path, monkeypatch, initialization_failure
):
    definition = load_robot("package://robo_arch/robots/ur7e/robot.yaml")
    initial = tuple(q + 0.01 for q in definition.default_positions)
    scene_config = SceneConfiguration(
        robot_system=RobotSystem(
            source=Path("system.yaml"),
            name="",
            pose=Pose(),
            robots=(
                RobotInstance(
                    name="arm", model="ur7e", pose=Pose(), initial_positions=initial
                ),
            ),
        ),
        sensors_enabled=False,
        objects=(),
    )
    world = DrakeWorld()
    builder = DiagramBuilder()
    scene = build_scene(scene_config, world, builder=builder)
    assert scene.plant.is_finalized()
    assert scene.initial_positions["arm"] == initial
    calls = []

    def configure(builder, scene):
        calls.append(scene)
        effort = builder.AddSystem(ConstantVectorSource(np.zeros(6)))
        builder.Connect(
            effort.get_output_port(),
            scene.plant.get_actuation_input_port(scene.robots["arm"]),
        )
        return {"external/state": effort.get_output_port()}

    # The world does not interpret task/controller names or import a scenario.
    run = RunConfiguration(
        source=Path("scenario.yaml"),
        world_config=world,
        duration=0.0035,
        robot_system=scene_config.robot_system,
        sensors_enabled=False,
        objects=(),
        task=TaskSelection(type="external_task"),
        autonomy=AutonomySelection(controller="external_control"),
    )
    trace_path = tmp_path / "trace.npz"
    recording = tmp_path / "scene.html"
    run = replace(
        run,
        world_config=world.model_copy(
            update={
                "visualization": world.visualization.model_copy(
                    update={"mode": "record", "open_browser": False}
                )
            }
        ),
    )
    advances = []
    advance = Simulator.AdvanceTo

    def advance_once(self, boundary_time):
        advances.append(boundary_time)
        return advance(self, boundary_time)

    def reject_forced_publish(*args):
        pytest.fail("Simulation visualization must use scheduled publish events")

    monkeypatch.setattr(Simulator, "AdvanceTo", advance_once)
    monkeypatch.setattr(Diagram, "ForcedPublish", reject_forced_publish)
    if initialization_failure:
        initialize = Simulator.Initialize

        def fail_after_initialization(self):
            if initialization_failure == "after":
                initialize(self)
            raise ValueError("controller initialization failed")

        monkeypatch.setattr(Simulator, "Initialize", fail_after_initialization)
        with pytest.raises(ValueError, match="controller initialization failed"):
            run_scenario(
                run, configure=configure, recording=recording, trace_path=trace_path
            )
        assert not advances
        assert trace_path.is_file() and recording.is_file()
        with np.load(trace_path) as trace:
            if initialization_failure == "after":
                np.testing.assert_allclose(trace["arm/q"][0], initial)
            else:
                assert trace["arm/q"].shape == (0, 6)
            np.testing.assert_allclose(trace["external/state"], 0)
            np.testing.assert_allclose(
                trace["external/state/times"],
                [0.0] if initialization_failure == "after" else [],
            )
        return
    result = run_scenario(
        run, configure=configure, recording=recording, trace_path=trace_path
    )
    trace = result["trace"]
    assert result["simulation_wall_seconds"] > 0
    assert result["realtime_rate"] * result["simulation_wall_seconds"] == pytest.approx(
        run.duration
    )
    assert len(calls) == 1
    assert advances == [run.duration]
    np.testing.assert_allclose(trace["arm/q"][0], initial)
    np.testing.assert_allclose(trace["arm/effort"], 0)
    np.testing.assert_allclose(trace["external/state"], 0)
    np.testing.assert_array_equal(trace["external/state/times"], trace["times"])
    np.testing.assert_allclose(trace["times"], [0.0, 0.001, 0.002, 0.003, 0.0035])
    assert trace_path.is_file() and recording.is_file()
    print(f"Inspect the actual run: python -m webbrowser {recording.as_uri()}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
