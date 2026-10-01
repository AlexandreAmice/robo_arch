"""A new robot needs only a declaration and assets, with no Python package."""

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from robo_arch.core.config import loading
from robo_arch.core.config.declarations import (
    Pose,
    RobotInstance,
    RobotSystem,
    SceneConfiguration,
)
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.urdf import compose


@pytest.fixture
def asset_scene(tmp_path, monkeypatch):
    # Deliberately no __init__.py, definition.py, or world-specific modules.
    robot = tmp_path / "robots" / "declarative_arm"
    asset = robot / "physical" / "arm.urdf"
    asset.parent.mkdir(parents=True)
    asset.write_text(
        """<robot name="arm">
  <link name="base"/>
  <link name="tip">
    <inertial>
      <mass value="1"/>
      <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>
  <joint name="hinge" type="revolute">
    <parent link="base"/><child link="tip"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="2"/>
  </joint>
  <transmission name="drive">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="hinge">
      <hardwareInterface>EffortJointInterface</hardwareInterface>
    </joint>
    <actuator name="motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>
</robot>""",
        encoding="utf-8",
    )
    (robot / "robot.yaml").write_text(
        "asset: package://robo_arch/robots/declarative_arm/physical/arm.urdf\n"
        "base_frame: base\njoints: [hinge]\ndefault_positions: [0.2]\n",
        encoding="utf-8",
    )
    monkeypatch.setattr(loading, "files", lambda package: tmp_path)
    monkeypatch.chdir(tmp_path.parent)
    return SceneConfiguration(
        robot_system=RobotSystem(
            name="",
            source=Path("unused.yaml"),
            pose=Pose(),
            robots=(
                RobotInstance(
                    name="arm",
                    model="declarative_arm",
                    pose=Pose(),
                    initial_positions=None,
                ),
            ),
        ),
        sensors_enabled=False,
        objects=(),
    )


@pytest.mark.parametrize("world", ["drake", "isaac"])
def test_data_only_robot_discovery_and_composition(asset_scene, tmp_path, world):
    definitions = load_definitions(asset_scene, world)
    definition = definitions.robots["declarative_arm"]
    assert definition.joints == ("hinge",)
    assert definition.default_positions == (0.2,)
    destination = tmp_path / "assembly.urdf"
    compose(resolve_devices(asset_scene).robots[0], (), definitions, destination)
    model = ET.parse(destination).getroot()
    assert [link.get("name") for link in model.findall("link")] == ["base", "tip"]
    assert model.find("joint").get("name") == "hinge"


@pytest.mark.parametrize(
    ("change", "message"),
    [
        ("asset: ./physical/arm.urdf", "package://robo_arch"),
        ("asset: package://robo_arch/../arm.urdf", "path traversal"),
        ("joints: [hinge, hinge]", "unique"),
        ("default_positions: []", "match the declared joint order"),
        ("default_positions: [.nan]", "finite"),
        ("factory: some.module", "Extra inputs"),
    ],
)
def test_invalid_asset_declarations_fail_without_sdks(
    asset_scene, tmp_path, change, message
):
    path = tmp_path / "robots/declarative_arm/robot.yaml"
    key = change.split(":", 1)[0]
    lines = [line for line in path.read_text().splitlines() if not line.startswith(key)]
    path.write_text("\n".join([*lines, change]) + "\n")
    with pytest.raises(ValueError, match=message):
        load_definitions(asset_scene, "drake")


def test_duplicate_declaration_keys_fail(asset_scene, tmp_path):
    path = tmp_path / "robots/declarative_arm/robot.yaml"
    path.write_text(path.read_text() + "base_frame: other\n")
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        load_definitions(asset_scene, "drake")


@pytest.mark.parametrize("world", ["drake", "isaac"])
def test_unsupported_asset_format_fails_before_construction(
    asset_scene, tmp_path, world
):
    path = tmp_path / "robots/declarative_arm/robot.yaml"
    path.write_text(path.read_text().replace("arm.urdf", "arm.unsupported"))
    with pytest.raises(ValueError, match=f"Robot arm has no {world} implementation"):
        load_definitions(asset_scene, world)


def test_missing_asset_fails_before_construction(asset_scene, tmp_path):
    (tmp_path / "robots/declarative_arm/physical/arm.urdf").unlink()
    with pytest.raises(FileNotFoundError, match="Robot arm asset does not exist"):
        load_definitions(asset_scene, "drake")


def test_asset_does_not_imply_a_hardware_driver(asset_scene):
    with pytest.raises(ValueError, match="Robot arm has no real implementation"):
        load_definitions(asset_scene, "real")


def test_data_only_robot_loads_in_drake(asset_scene):
    pytest.importorskip("pydrake")
    from pydrake.multibody.plant import MultibodyPlant

    from robo_arch.core.worlds.drake.models import add_robot

    definition = load_definitions(asset_scene, "drake").robots["declarative_arm"]
    plant = MultibodyPlant(0.001)
    left, right = [
        add_robot(plant, definition, name=name) for name in ("left", "right")
    ]
    for instance in (left, right):
        plant.WeldFrames(
            plant.world_frame(), plant.GetFrameByName(definition.base_frame, instance)
        )
    plant.Finalize()
    context = plant.CreateDefaultContext()
    plant.SetPositions(context, left, [0.5])
    plant.SetPositions(context, right, definition.default_positions)
    assert plant.GetPositions(context, left)[0] == 0.5
    assert plant.GetPositions(context, right)[0] == 0.2


def test_drake_rejects_incorrect_declared_joint_order(asset_scene):
    pytest.importorskip("pydrake")
    from pydrake.multibody.plant import MultibodyPlant

    from robo_arch.core.worlds.drake.models import add_robot

    definition = load_definitions(asset_scene, "drake").robots["declarative_arm"]
    definition = definition.model_copy(update={"joints": ("wrong_joint",)})
    with pytest.raises(ValueError, match="Robot arm actuator order"):
        add_robot(MultibodyPlant(0.001), definition, name="arm")


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
