"""Ideal camera intrinsics and supported factories, without simulator imports."""

from math import pi

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters
from robo_arch.core.worlds.registry import FactoryReference, SensorDefinition


class CameraParameters(Parameters):
    """Co-located RGB/depth cameras: +z forward, +x right, +y down."""

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


# Intentional approximation: ideal pinhole RGB-D without noise or latency.
IMPLEMENTATIONS = {
    "drake": FactoryReference(
        module="robo_arch.sensors.ideal_camera.drake",
        attribute="add_to_builder",
    ),
}

DEFINITION = SensorDefinition(
    parameter_schema=CameraParameters, implementations=IMPLEMENTATIONS
)
