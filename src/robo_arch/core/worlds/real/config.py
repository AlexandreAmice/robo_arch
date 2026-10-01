"""SDK-independent Real world settings."""

from typing import Literal

from pydantic import Field, field_validator

from robo_arch.core.config.resources import validate_package_reference
from robo_arch.core.config.schema import Schema


class Ros2Transport(Schema):
    type: Literal["ros2"] = "ros2"
    namespace: str = "/cell"
    clock: Literal["system", "ros"] = "system"


class Rviz2Visualization(Schema):
    type: Literal["rviz2"] = "rviz2"
    mode: Literal["off", "live"] = "off"
    fixed_frame: str = Field(default="world", min_length=1)
    config: str | None = None

    @field_validator("config")
    @classmethod
    def package_config(cls, value: str | None) -> str | None:
        return validate_package_reference(value) if value is not None else None


class RealWorld(Schema):
    """Hardware declarations; these do not establish device execution support."""

    type: Literal["real"] = "real"
    transport: Ros2Transport = Field(default_factory=Ros2Transport)
    visualization: Rviz2Visualization = Field(default_factory=Rviz2Visualization)
