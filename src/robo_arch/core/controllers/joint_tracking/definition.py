"""Validated joint-tracking gains without simulation imports."""

import math
from typing import Self

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters


class JointTrackingParameters(Parameters):
    """Acceleration-feedback gains: kp in s^-2 and kd in s^-1."""

    kp: tuple[float, ...] = Field(min_length=1)
    kd: tuple[float, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def validate_gains(self) -> Self:
        if len(self.kp) != len(self.kd):
            raise ValueError("kp and kd must have the same number of joints")
        if not all(math.isfinite(gain) and gain > 0 for gain in (*self.kp, *self.kd)):
            raise ValueError("kp and kd must contain finite positive gains")
        return self
