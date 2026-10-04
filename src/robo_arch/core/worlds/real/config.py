"""SDK-independent Real world settings."""

from typing import Literal

from pydantic import Field, field_validator

from robo_arch.core.config.resources import validate_package_reference
from robo_arch.core.config.schema import Schema


class Ros2Transport(Schema):
    """Declared ROS 2 transport selection; no nodes or drivers are started.

    :param type: Fixed discriminator ``ros2``.
    :param namespace: Requested ROS namespace, default /cell. This schema does
        not validate all ROS naming rules or assign hardware identities.
    :param clock: system or ros time source selection.

    Unknown fields/discriminators raise ValidationError. Transport execution and
    hardware drivers remain unimplemented by these declarations.
    """

    type: Literal["ros2"] = "ros2"
    namespace: str = "/cell"
    clock: Literal["system", "ros"] = "system"


class Rviz2Visualization(Schema):
    """RViz launcher settings, independent of device execution.

    :param type: Fixed discriminator ``rviz2``.
    :param mode: off or live.
    :param fixed_frame: Nonempty frame name, default world.
    :param config: Optional application-package URI to an RViz configuration.
        Syntax is validated here; the launcher checks file availability.

    Invalid fields raise ValidationError. The launcher owns only the viewer;
    ROS publishers and hardware drivers must be supplied separately.
    """

    type: Literal["rviz2"] = "rviz2"
    mode: Literal["off", "live"] = "off"
    fixed_frame: str = Field(default="world", min_length=1)
    config: str | None = None

    @field_validator("config")
    @classmethod
    def package_config(cls, value: str | None) -> str | None:
        return validate_package_reference(value) if value is not None else None


class RealWorld(Schema):
    """Hardware declarations; these do not establish device execution support.

    :param type: Fixed discriminator ``real``.
    :param transport: Ros2Transport selection, populated with defaults if omitted.
    :param visualization: Rviz2Visualization selection, off by default.

    There is no simulated physics time step. The current real-world scene/run
    entry points raise NotImplementedError; RViz can be launched independently.
    Invalid or foreign fields raise ValidationError without importing ROS SDKs.
    """

    type: Literal["real"] = "real"
    transport: Ros2Transport = Field(default_factory=Ros2Transport)
    visualization: Rviz2Visualization = Field(default_factory=Rviz2Visualization)
