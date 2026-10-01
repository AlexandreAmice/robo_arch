"""Reject lossy SDF translation before opening a simulator."""

from dataclasses import replace
from importlib.resources import files
from pathlib import Path

import pytest

from robo_arch.core.config.declarations import (
    AutonomySelection,
    ObjectInstance,
    Pose,
    RobotInstance,
    RobotSystem,
    RunConfiguration,
    SceneConfiguration,
    TaskSelection,
)
from robo_arch.core.worlds.devices import DeviceDefinitions
from robo_arch.core.worlds.isaac import objects
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.scenario import run_scenario
from robo_arch.objects.box.definition import describe


@pytest.fixture
def fixture_scene():
    return SceneConfiguration(
        robot_system=RobotSystem(source=Path("system.yaml"), name="", pose=Pose()),
        sensors_enabled=False,
        objects=(ObjectInstance(name="fixture", model="box", pose=Pose()),),
    )


def definitions():
    return DeviceDefinitions(robots={}, sensors={}, objects={"box": describe()})


def test_packaged_fixture_preserves_dimensions_color_and_instance(fixture_scene):
    (box,) = objects.load_boxes(fixture_scene, definitions())
    assert box.size == (0.05, 0.05, 0.05)
    assert box.rgba == (0.9, 0.45, 0.1, 1.0)
    assert box.instance == fixture_scene.objects[0]


@pytest.mark.parametrize(
    ("old", "new"),
    [
        ("</link>", '<visual name="extra"/></link>'),
        ("</link>", '<collision name="extra"/></link>'),
        ("</model>", '<link name="extra"/></model>'),
        ("</collision>", "<surface><friction/></surface></collision>"),
        ("</visual>", "<pose>0 0 0 0 0 0</pose></visual>"),
        ("<box>", "<sphere>"),
        ('name="box">\n      <inertial>', 'name="other">\n      <inertial>'),
        ("0.05 0.05 0.05", "nan 0.05 0.05"),
        ("<diffuse>", "<ambient>"),
    ],
)
def test_unsupported_content_fails_before_kit(
    fixture_scene, tmp_path, monkeypatch, old, new
):
    data = files("robo_arch.objects.box").joinpath("model.sdf").read_text()
    # Keep XML well-formed when replacing a geometry/material tag.
    modified = data.replace(old, new)
    if old in ("<box>", "<diffuse>"):
        modified = modified.replace(old.replace("<", "</"), new.replace("<", "</"))
    (tmp_path / "model.sdf").write_text(modified)
    monkeypatch.setattr(objects, "files", lambda package: tmp_path)
    robot_system = replace(
        fixture_scene.robot_system,
        robots=(
            RobotInstance(
                name="arm", model="ur7e", pose=Pose(), initial_positions=None
            ),
        ),
    )
    run = RunConfiguration(
        source=Path("scenario.yaml"),
        world_config=IsaacWorld(),
        duration=0.01,
        robot_system=robot_system,
        objects=fixture_scene.objects,
        sensors_enabled=False,
        task=TaskSelection(type="unused"),
        autonomy=AutonomySelection(controller="unused"),
    )
    # This environment does not need Kit; rejection precedes its optional import.
    with pytest.raises(ValueError, match="Isaac fixture fixture:"):
        run_scenario(run, configure=lambda scene: {})


def test_numeric_equivalence_does_not_depend_on_xml_whitespace(
    fixture_scene, tmp_path, monkeypatch
):
    data = files("robo_arch.objects.box").joinpath("model.sdf").read_text()
    (tmp_path / "model.sdf").write_text(
        data.replace("0.05 0.05 0.05", "5e-2  .05\n .05", 1)
    )
    monkeypatch.setattr(objects, "files", lambda package: tmp_path)
    assert objects.load_boxes(fixture_scene, definitions())[0].size == (0.05,) * 3


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
