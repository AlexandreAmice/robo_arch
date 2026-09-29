"""Reject invalid controller tuning before a simulation is constructed."""

import pytest
from pydantic import ValidationError

from robo_arch.core.controllers.joint_tracking.definition import (
    JointTrackingParameters,
)


@pytest.mark.parametrize(
    ("kp", "kd"),
    [([], []), ([1.0], [1.0, 2.0]), ([float("nan")], [1.0]), ([1.0], [0.0])],
)
def test_invalid_feedback_gains_are_rejected(kp, kd):
    with pytest.raises(ValidationError):
        JointTrackingParameters(kp=kp, kd=kd)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
