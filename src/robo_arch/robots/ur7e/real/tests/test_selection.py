"""Reject mismatched command/calibration identities without importing ROS."""

from dataclasses import replace

import pytest

from robo_arch.core.config.declarations import CalibrationProfile, DeviceBinding, Pose
from robo_arch.core.contracts.commands import CommandKind
from robo_arch.core.worlds.assembly import PlacedRobot
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.robots.ur7e.real import UrTrajectoryAdapter, validate_selection


def robot():
    return PlacedRobot(
        name="left/arm",
        model="ur7e",
        poses=(Pose(),),
        initial_positions=None,
        binding=DeviceBinding(identity="mock-left", endpoint="/left"),
        calibration=CalibrationProfile(
            identity="mock-left",
            parent_frame="world",
            child_frame="base_link",
            mounting_revision="v1",
            kind="synthetic",
            pose=Pose(),
        ),
        mounting_revision="v1",
    )


def test_effort_policy_rejected_before_ros_import():
    with pytest.raises(ValueError, match="Unsupported command"):
        UrTrajectoryAdapter(robot(), RealWorld(), CommandKind.JOINT_EFFORT)


def test_explicit_synthetic_identity_does_not_require_serial():
    instance = robot()
    names = validate_selection(instance, CommandKind.JOINT_POSITION_TRAJECTORY)
    assert len(names) == 6 and names[0] == "shoulder_pan_joint"
    assert instance.binding.physical_id is None


@pytest.mark.parametrize("change", ["identity", "frame", "revision", "absent"])
def test_invalid_calibration_fails(change):
    instance = robot()
    values = {
        "identity": {"identity": "other"},
        "frame": {"child_frame": "tool0"},
        "revision": {"mounting_revision": "v2"},
    }
    instance = replace(
        instance,
        calibration=None
        if change == "absent"
        else instance.calibration.model_copy(update=values[change]),
    )
    with pytest.raises(ValueError):
        validate_selection(instance, CommandKind.JOINT_POSITION_TRAJECTORY)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
