"""World-owned assembly and execution without a concrete task implementation."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pydrake")
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


def test_scene_and_scenario_have_independent_entry_points(tmp_path, monkeypatch):
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
    result = run_scenario(
        run, configure=configure, recording=recording, trace_path=trace_path
    )
    trace = result["trace"]
    assert len(calls) == 1
    assert advances == [run.duration]
    np.testing.assert_allclose(trace["arm/q"][0], initial)
    np.testing.assert_allclose(trace["arm/effort"], 0)
    np.testing.assert_allclose(trace["times"], [0.0, 0.001, 0.002, 0.003, 0.0035])
    assert trace_path.is_file() and recording.is_file()
    print(f"Inspect the actual run: python -m webbrowser {recording.as_uri()}")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
