"""SDK-independent Drake world settings."""

from typing import Annotated, Literal

from pydantic import Field

from robo_arch.core.config.schema import Schema

type _ColorChannel = Annotated[float, Field(ge=0, le=1)]
type _Rgba = tuple[_ColorChannel, _ColorChannel, _ColorChannel, _ColorChannel]


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
    """VisualizationConfig display fields plus viewer lifecycle controls.

    Colors are RGBA tuples in [0, 1], converted to SDK types during wiring.
    The world owns Meshcat creation and transport; mouse forces stay disabled.
    """

    type: Literal["meshcat"] = "meshcat"
    mode: Literal["off", "live", "record", "live_and_record"] = "off"
    open_browser: bool = True
    publish_period: float = Field(default=1 / 64, gt=0)
    publish_illustration: bool = True
    default_illustration_color: _Rgba = (0.9, 0.9, 0.9, 1.0)
    publish_proximity: bool = True
    default_proximity_color: _Rgba = (0.8, 0.0, 0.0, 1.0)
    initial_proximity_alpha: float = Field(default=0.5, ge=0, le=1)
    publish_contacts: bool = True
    publish_inertia: bool = True
    delete_on_initialization_event: bool = True
    enable_alpha_sliders: bool = False


class DrakeWorld(Schema):
    type: Literal["drake"] = "drake"
    # Physical floor at world z=0, independent of viewer selection.
    ground: bool = True
    # Wall-clock pacing is independent of the physics and viewer publication step.
    target_realtime_rate: float = Field(default=0.0, ge=0)
    physics: DrakePhysics = Field(default_factory=DrakePhysics)
    visualization: DrakeVisualization = Field(default_factory=DrakeVisualization)
