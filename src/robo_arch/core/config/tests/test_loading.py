"""Package-resource loading, recursive instances and actionable schema failures."""

from pathlib import Path

import pytest

from robo_arch.core.config import loading
from robo_arch.core.config.loading import load_run


@pytest.fixture(autouse=True)
def package_resources(tmp_path, monkeypatch):
    monkeypatch.setattr(loading, "files", lambda package: tmp_path)


def _bundle(root: Path) -> Path:
    (root / "systems").mkdir()
    files = {
        "scenario.yaml": """world: {type: drake, physics: {time_step: 0.001}}
duration: 1.0
robot_system:
  definition: package://robo_arch/systems/pair.yaml
  pose: {translation: [1, 0, 0]}
  autonomy: {controller: joint_tracking, parameters: {kp: [100]}}
objects:
  box: {model: box, pose: {translation: [0.4, 0, 0.1]}}
task: {type: joint_tracking, parameters: {robot: left/arm}}
""",
        "systems/pair.yaml": """systems:
  left:
    definition: package://robo_arch/systems/arm.yaml
    pose: {translation: [0, 1, 0]}
  right:
    definition: package://robo_arch/systems/arm.yaml
    pose: {translation: [0, -1, 0]}
""",
        "systems/arm.yaml": """robots:
  arm: {model: example_arm}
sensors:
  camera:
    model: ideal_camera
    parent: arm/tool0
    pose: {translation: [0, 0, 0.08]}
""",
    }
    for name, content in files.items():
        (root / name).write_text(content, encoding="utf-8")
    return root / "scenario.yaml"


def test_nested_instances_keep_namespace_and_pose_chain(tmp_path, monkeypatch):
    run_path = _bundle(tmp_path)
    monkeypatch.chdir(tmp_path.parent)
    run = load_run("package://robo_arch/scenario.yaml")
    assert [robot.name for robot in run.robots] == ["left/arm", "right/arm"]
    assert [sensor.parent for sensor in run.sensors] == [
        "left/arm/tool0",
        "right/arm/tool0",
    ]
    assert run.robots[0].poses[0].translation == (1, 0, 0)
    assert run.robots[0].poses[1].translation == (0, 1, 0)
    assert run.robots[1].poses[1].translation == (0, -1, 0)
    assert run.sensors[0].pose.translation == (0, 0, 0.08)
    assert run.sensors[0].parameters is not run.sensors[1].parameters
    assert run.resources.count(tmp_path / "systems/arm.yaml") == 1
    assert len(run.resources) == 3
    assert run.objects[0].pose.translation == (0.4, 0, 0.1)
    assert run.autonomy.parameters["kp"] == [100]
    assert run.source == run_path


@pytest.mark.parametrize(
    ("file", "content", "message"),
    [
        ("scenario.yaml", "objects: {}\nobjects: {}\n", "Duplicate YAML key"),
        ("scenario.yaml", "objects: {}\nunknown: true\n", "Extra inputs"),
        (
            "systems/pair.yaml",
            "robots: {box: {model: example_arm}}\n",
            "Object and device names must be distinct",
        ),
        (
            "systems/arm.yaml",
            "systems: {again: {definition: package://robo_arch/systems/pair.yaml}}\n",
            "Recursive robot system inclusion",
        ),
        (
            "systems/arm.yaml",
            "sensors: {camera: {model: ideal_camera, parent: missing/tool0}}\n",
            "must name a robot/body",
        ),
        (
            "systems/arm.yaml",
            "robots: {arm: {model: example_arm}}\n"
            "sensors: {arm: {model: ideal_camera, parent: arm/tool0}}\n",
            "names must be unique",
        ),
        (
            "systems/arm.yaml",
            "systems: {missing: {definition: package://robo_arch/systems/absent.yaml}}\n",
            "absent.yaml",
        ),
    ],
)
def test_invalid_bundle_fails_before_world_construction(
    tmp_path, file, content, message
):
    run_path = _bundle(tmp_path)
    (tmp_path / file).write_text(content, encoding="utf-8")
    with pytest.raises(ValueError, match=message):
        load_run(run_path)


@pytest.mark.parametrize(
    "reference",
    [
        "scenario.yaml",
        "../scenario.yaml",
        "/tmp/scenario.yaml",
        "",
        "package://unknown/scenario.yaml",
        "package://robo_arch/../scenario.yaml",
        "package://robo_arch//scenario.yaml",
        "package://robo_arch/./scenario.yaml",
        "package://robo_arch/%2e%2e/scenario.yaml",
        "package://robo_arch/scenario.yaml?variant=1",
        "package://robo_arch/scenario.yaml#part",
    ],
)
def test_file_references_and_ambiguous_uris_are_rejected(tmp_path, reference):
    run_path = _bundle(tmp_path)
    run_path.write_text(
        run_path.read_text().replace(
            "package://robo_arch/systems/pair.yaml", f"'{reference}'"
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="must use package://robo_arch"):
        load_run(run_path)


def test_scenario_can_move_without_changing_references(tmp_path):
    run_path = _bundle(tmp_path)
    destination = tmp_path / "elsewhere" / "moved.yaml"
    destination.parent.mkdir()
    run_path.rename(destination)
    assert load_run(destination).task.parameters["robot"] == "left/arm"


def test_sensors_can_be_disabled_without_changing_the_system(tmp_path):
    scenario = _bundle(tmp_path)
    scenario.write_text(scenario.read_text() + "sensors_enabled: false\n")
    run = load_run(scenario)
    assert run.sensors == ()
    assert len(run.robots) == 2
    assert len(run.objects) == 1


def test_world_profile_resolves_from_package_after_scenario_moves(tmp_path):
    scenario = _bundle(tmp_path)
    profile = tmp_path / "world.yaml"
    profile.write_text(
        "type: isaac\nphysics: {time_step: 0.002, solver: pgs, device: cpu}\n"
        "visualization: {mode: live, collision_geometry: true}\n"
    )
    scenario.write_text(
        scenario.read_text().replace(
            "world: {type: drake, physics: {time_step: 0.001}}",
            "world: package://robo_arch/world.yaml",
        )
    )
    moved = tmp_path / "elsewhere.yaml"
    scenario.rename(moved)
    run = load_run(moved)
    assert run.world == "isaac"
    assert run.time_step == 0.002
    assert run.world_config.physics.solver == "pgs"
    assert profile in run.resources
    assert loading.load_world("package://robo_arch/world.yaml") == run.world_config


@pytest.mark.parametrize(
    "reference",
    ["drake", "world.yaml", "/tmp/world.yaml", "package://other/world.yaml"],
)
def test_world_selection_rejects_bare_names_and_file_references(tmp_path, reference):
    scenario = _bundle(tmp_path)
    scenario.write_text(
        scenario.read_text().replace(
            "world: {type: drake, physics: {time_step: 0.001}}",
            f"world: {reference}",
        )
    )
    with pytest.raises(ValueError, match="must use package://robo_arch"):
        load_run(scenario)


def test_world_profile_does_not_accept_inheritance_or_duplicate_keys(tmp_path):
    profile = tmp_path / "world.yaml"
    profile.write_text("type: drake\ntype: isaac\n")
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        loading.load_world(profile)
    profile.write_text("type: drake\nextends: package://robo_arch/base.yaml\n")
    with pytest.raises(ValueError, match="Extra inputs"):
        loading.load_world(profile)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
