"""SDK-independent Drake world settings."""

from typing import Literal

from pydantic import Field

from robo_arch.core.config.schema import Schema


class DrakePhysics(Schema):
    """Discrete plant settings; the time step is in seconds."""

    time_step: float = Field(default=0.001, gt=0)
    contact_model: Literal["point", "hydroelastic", "hydroelastic_with_fallback"] = (
        "hydroelastic_with_fallback"
    )
    discrete_contact_approximation: Literal["sap", "similar", "lagged"] = "sap"

    # Dimensionless SAP near-rigid regularization; zero disables it.
    sap_near_rigid_threshold: float = Field(default=1.0, ge=0)


class DrakeVisualization(Schema):
    type: Literal["meshcat"] = "meshcat"
    mode: Literal["off", "live", "record", "live_and_record"] = "off"
    publish_illustration: bool = True
    publish_proximity: bool = True
    publish_contacts: bool = True
    publish_inertia: bool = True
    publish_period: float = Field(default=1 / 64, gt=0)
    open_browser: bool = True


class DrakeWorld(Schema):
    type: Literal["drake"] = "drake"
    # Wall-clock pacing is independent of the physics and viewer publication step.
    target_realtime_rate: float = Field(default=0.0, ge=0)
    physics: DrakePhysics = Field(default_factory=DrakePhysics)
    visualization: DrakeVisualization = Field(default_factory=DrakeVisualization)
