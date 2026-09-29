"""Resolve nested physical systems without importing simulator SDKs."""

from dataclasses import replace
from pathlib import Path

import pytest

from robo_arch.core.config.declarations import (
    AutonomySelection,
    ObjectInstance,
    Pose,
    RobotInstance,
    RobotSystem,
    RunConfiguration,
    SensorInstance,
    TaskSelection,
)
from robo_arch.core.worlds.assembly import resolve_devices


def _run() -> RunConfiguration:
    arm = RobotSystem(
        name="left",
        source=Path("arm.yaml"),
        pose=Pose(translation=(0, 1, 0), rpy=(0, 0, 0.5)),
        robots=(
            RobotInstance(
                name="arm", model="example_arm", pose=Pose(), initial_positions=None
            ),
        ),
        sensors=(
            SensorInstance(
                name="camera",
                model="ideal_camera",
                parent="arm/tool0",
                pose=Pose(translation=(0, 0, 0.08)),
                parameters={},
            ),
        ),
    )
    return RunConfiguration(
        source=Path("scenario.yaml"),
        world="drake",
        duration=1,
        time_step=0.001,
        robot_system=RobotSystem(
            name="",
            source=Path("pair.yaml"),
            pose=Pose(translation=(1, 0, 0)),
            systems=(
                arm,
                replace(arm, name="right", pose=Pose(translation=(0, -1, 0))),
            ),
        ),
        sensors_enabled=True,
        objects=(),
        task=TaskSelection(type="tracking"),
        autonomy=AutonomySelection(controller="joint_tracking"),
    )


def test_namespace_pose_order_and_relative_sensor_mounts():
    run = _run()
    devices = resolve_devices(run)
    assert [robot.name for robot in devices.robots] == ["left/arm", "right/arm"]
    assert [sensor.name for sensor in devices.sensors] == [
        "left/camera",
        "right/camera",
    ]
    assert [sensor.parent for sensor in devices.sensors] == [
        "left/arm/tool0",
        "right/arm/tool0",
    ]
    for robot, system in zip(devices.robots, run.robot_system.systems, strict=True):
        assert robot.poses == (
            run.robot_system.pose,
            system.pose,
            system.robots[0].pose,
        )
    assert devices.sensors[0].pose.translation == (0, 0, 0.08)
    assert run.robot_system.systems[0].robots[0].name == "arm"
    assert run.robot_system.systems[0].sensors[0].parent == "arm/tool0"


def test_disabling_sensors_preserves_declared_composition():
    run = replace(_run(), sensors_enabled=False)
    devices = resolve_devices(run)
    assert devices.sensors == ()
    assert len(devices.robots) == 2
    assert len(run.robot_system.systems[0].sensors) == 1


@pytest.mark.parametrize("enabled", [True, False])
def test_missing_sensor_parent_fails_even_when_disabled(enabled):
    run = _run()
    child = run.robot_system.systems[0]
    child = replace(child, sensors=(replace(child.sensors[0], parent="missing/tool0"),))
    run = replace(
        run,
        robot_system=replace(run.robot_system, systems=(child,)),
        sensors_enabled=enabled,
    )
    with pytest.raises(ValueError, match="left/camera.*left/missing/tool0"):
        resolve_devices(run)


def test_parent_can_reference_a_nested_robot():
    run = _run()
    child = run.robot_system.systems[0]
    sensor = replace(child.sensors[0], name="external_camera", parent="left/arm/tool0")
    run = replace(run, robot_system=replace(run.robot_system, sensors=(sensor,)))
    assert resolve_devices(run).sensors[0].parent == "left/arm/tool0"


def test_object_device_name_conflict_fails():
    run = _run()
    child = replace(run.robot_system.systems[0], name="", robots=())
    run = replace(
        run,
        robot_system=child,
        objects=(ObjectInstance(name="camera", model="box", pose=Pose()),),
    )
    with pytest.raises(ValueError, match="Object and device names must be distinct"):
        resolve_devices(run)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
