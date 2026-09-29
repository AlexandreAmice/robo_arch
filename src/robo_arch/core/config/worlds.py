"""SDK-independent settings consumed by each world's native implementation."""

from typing import Annotated, Literal

from pydantic import BaseModel, ConfigDict, Field, TypeAdapter, field_validator


class _Schema(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True, allow_inf_nan=False)


def validate_package_reference(reference: str) -> str:
    """Reject filesystem paths and ambiguous application-package resource URIs."""
    prefix = "package://robo_arch/"
    parts = reference.removeprefix(prefix).split("/")
    if (
        not reference.startswith(prefix)
        or any(part in {"", ".", ".."} for part in parts)
        or any(character in reference for character in "\\%?#")
    ):
        raise ValueError(
            f"Reference {reference!r} must use "
            "package://robo_arch/<resource> without path traversal"
        )
    return reference


class DrakePhysics(_Schema):
    """Discrete plant settings; the time step is in seconds."""

    time_step: float = Field(default=0.001, gt=0)
    contact_model: Literal["point", "hydroelastic", "hydroelastic_with_fallback"] = (
        "hydroelastic_with_fallback"
    )
    discrete_contact_approximation: Literal["sap", "similar", "lagged"] = "sap"

    # Dimensionless SAP near-rigid regularization; zero disables it.
    sap_near_rigid_threshold: float = Field(default=1.0, ge=0)


class DrakeVisualization(_Schema):
    type: Literal["meshcat"] = "meshcat"
    mode: Literal["off", "live", "record", "live_and_record"] = "off"
    publish_illustration: bool = True
    publish_proximity: bool = True
    publish_contacts: bool = True
    publish_inertia: bool = True
    publish_period: float = Field(default=1 / 64, gt=0)
    open_browser: bool = True


class DrakeWorld(_Schema):
    type: Literal["drake"] = "drake"
    # Wall-clock pacing is independent of the physics and viewer publication step.
    target_realtime_rate: float = Field(default=0.0, ge=0)
    physics: DrakePhysics = Field(default_factory=DrakePhysics)
    visualization: DrakeVisualization = Field(default_factory=DrakeVisualization)


class IsaacPhysics(_Schema):
    """Native PhysX scene settings; the time step is in seconds."""

    time_step: float = Field(default=0.001, gt=0)
    solver: Literal["tgs", "pgs"] = "tgs"
    device: Literal["cpu", "cuda:0"] = "cuda:0"


class IsaacVisualization(_Schema):
    type: Literal["isaac"] = "isaac"
    mode: Literal["off", "live"] = "off"
    collision_geometry: bool = False
    publish_period: float = Field(default=1 / 30, gt=0)


class IsaacWorld(_Schema):
    type: Literal["isaac"] = "isaac"
    physics: IsaacPhysics = Field(default_factory=IsaacPhysics)
    visualization: IsaacVisualization = Field(default_factory=IsaacVisualization)


class Ros2Transport(_Schema):
    type: Literal["ros2"] = "ros2"
    namespace: str = "/cell"
    clock: Literal["system", "ros"] = "system"


class Rviz2Visualization(_Schema):
    type: Literal["rviz2"] = "rviz2"
    mode: Literal["off", "live"] = "off"
    fixed_frame: str = Field(default="world", min_length=1)
    config: str | None = None

    @field_validator("config")
    @classmethod
    def package_config(cls, value: str | None) -> str | None:
        return validate_package_reference(value) if value is not None else None


class RealWorld(_Schema):
    """Hardware declarations; these do not establish device execution support."""

    type: Literal["real"] = "real"
    transport: Ros2Transport = Field(default_factory=Ros2Transport)
    visualization: Rviz2Visualization = Field(default_factory=Rviz2Visualization)


WorldConfiguration = Annotated[
    DrakeWorld | IsaacWorld | RealWorld, Field(discriminator="type")
]
_WORLD_ADAPTER = TypeAdapter(WorldConfiguration)


def parse_world(value: object) -> WorldConfiguration:
    """Validate one complete native world configuration, without importing SDKs."""
    return _WORLD_ADAPTER.validate_python(value)
