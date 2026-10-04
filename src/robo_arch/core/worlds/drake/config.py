"""SDK-independent Drake world settings."""

from typing import Annotated, Literal

from pydantic import Field

from robo_arch.core.config.schema import Schema

type _ColorChannel = Annotated[float, Field(ge=0, le=1)]
type _Rgba = tuple[_ColorChannel, _ColorChannel, _ColorChannel, _ColorChannel]


class DrakePhysics(Schema):
    """Settings passed to the discrete Drake plant during world construction.

    :param time_step: Finite positive physics step in seconds (default 0.001).
    :param contact_model: Point, hydroelastic, or hydroelastic with point fallback.
    :param discrete_contact_approximation: SAP, similar or lagged approximation.
    :param sap_near_rigid_threshold: Nonnegative dimensionless SAP regularization;
        zero disables near-rigid regularization.

    Unknown fields and invalid values raise ValidationError. Importing this model
    requires no Drake SDK; physical applicability is checked during construction.
    """

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

    :param type: Fixed discriminator ``meshcat``.
    :param mode: off, live, record or live_and_record.
    :param open_browser: Whether the runner may open the viewer in a browser.
    :param publish_period: Positive simulation-time publication period, seconds.
    :param publish_illustration: Publish illustration geometry.
    :param default_illustration_color: Fallback RGBA for illustration geometry.
    :param publish_proximity: Publish the collision/proximity layer.
    :param default_proximity_color: Fallback RGBA for proximity geometry.
    :param initial_proximity_alpha: Initial proximity opacity in [0, 1].
    :param publish_contacts: Publish contact-force visualization.
    :param publish_inertia: Publish inertia visualization.
    :param delete_on_initialization_event: Clear published geometry at initialization.
    :param enable_alpha_sliders: Add native alpha controls where supported.

    Defaults are shown in the constructor signature. Invalid fields or colors
    raise ValidationError. These declarations do not create a viewer or change
    physical collision roles. Browser, recording and hold behavior belong to the
    runner; server-backed sliders are not included in standalone HTML recordings.
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
    """Complete Drake world profile, inspectable without importing Drake.

    :param type: Fixed discriminator ``drake``.
    :param ground: Construct the physical world-z=0 floor, independently of viewing.
    :param target_realtime_rate: Nonnegative simulated/wall-time pacing ratio;
        zero is unpaced. A positive value limits speed, not a performance promise.
    :param physics: DrakePhysics settings, populated from defaults when omitted.
    :param visualization: DrakeVisualization settings, off by default.

    Invalid fields raise ValidationError. This profile does not select the scene,
    task or autonomy and does not establish that an SDK is installed.
    """

    type: Literal["drake"] = "drake"
    # Physical floor at world z=0, independent of viewer selection.
    ground: bool = True
    # Wall-clock pacing is independent of the physics and viewer publication step.
    target_realtime_rate: float = Field(default=0.0, ge=0)
    physics: DrakePhysics = Field(default_factory=DrakePhysics)
    visualization: DrakeVisualization = Field(default_factory=DrakeVisualization)
