"""Pose conventions and composition shared by world implementations."""

import math
import subprocess
import sys

import numpy as np
import pytest

from robo_arch.core.config.declarations import Pose
from robo_arch.core.worlds.assembly import PlacedRobot, base_pose, pose_transform


def test_pose_uses_extrinsic_roll_pitch_yaw_and_parent_frame_translation():
    pose = Pose(translation=(1, 2, 3), rpy=(math.pi / 2, math.pi / 6, math.pi / 2))
    expected = [
        [0, 0, 1, 1],
        [math.sqrt(3) / 2, 0.5, 0, 2],
        [-0.5, math.sqrt(3) / 2, 0, 3],
        [0, 0, 0, 1],
    ]
    np.testing.assert_allclose(
        pose_transform(pose).GetAsMatrix4(), expected, atol=1e-15
    )


def test_nested_base_pose_rotates_child_translations_in_order():
    robot = PlacedRobot(
        name="nested/arm",
        model="example",
        initial_positions=None,
        poses=(
            Pose(translation=(1, 2, 3), rpy=(0, 0, math.pi / 2)),
            Pose(translation=(2, 0, 0), rpy=(math.pi / 2, 0, 0)),
            Pose(translation=(0, 3, 0)),
        ),
    )
    np.testing.assert_allclose(
        base_pose(robot).GetAsMatrix4(),
        [[0, 0, 1, 1], [1, 0, 0, 4], [0, 1, 0, 6], [0, 0, 0, 1]],
        atol=1e-15,
    )


def test_assembly_import_does_not_load_drake():
    script = f"""
import sys
sys.path[:] = {sys.path!r}
class RejectDrake:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] == 'pydrake':
            raise AssertionError('Unexpected Drake import: ' + fullname)
sys.meta_path.insert(0, RejectDrake())
from robo_arch.core.worlds.assembly import PlacedRobot, base_pose, pose_transform
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
