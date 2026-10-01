"""Ideal camera intrinsics and supported factories, without simulator imports."""

from math import pi

from pydantic import Field, model_validator

from robo_arch.core.config.declarations import SensorDefinition
from robo_arch.core.config.parameters import Parameters


class CameraParameters(Parameters):
    """Ideal RGB/depth intrinsics; optical +z forward, +x right, +y down."""

    width: int = Field(default=64, gt=0)
    height: int = Field(default=48, gt=0)
    vertical_fov_radians: float = Field(default=pi / 3, gt=0, lt=pi)
    near_m: float = Field(default=0.02, gt=0)
    far_m: float = Field(default=5.0, gt=0, allow_inf_nan=False)

    @model_validator(mode="after")
    def validate_range(self) -> "CameraParameters":
        if self.near_m >= self.far_m:
            raise ValueError("near_m must be less than far_m")
        return self


def describe() -> SensorDefinition:
    """Ideal pinhole RGB-D without noise or latency; currently supported in Drake."""
    return SensorDefinition(
        parameter_schema=CameraParameters,
        supported_worlds=("drake",),
        physical_worlds=("drake", "isaac"),
        kind="camera",
    )
