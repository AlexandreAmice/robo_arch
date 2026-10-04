"""SDK-independent selection and tuning for sphere-based protection."""

from typing import Literal, Self

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.resources import validate_package_reference


class ProtectionParameters(Parameters):
    """Protection selection and tuning, independent of nominal control.

    :param backend: drake (default) or optional torch_moreau CUDA execution.
    :param compile_model: Compile the shared tensor model; requires torch_moreau.
    :param velocity_limit_gain: Positive gain in s^-1 for velocity-bound barriers.
    :param profiles: Instance name to ``package://robo_arch/...`` sphere-profile
        URI. Instance names must match the physical assembly; treat as read-only.
    :param protected: Nonempty unique instances whose spheres require protection;
        each must have a profile. Pairs within one instance are not generated.
    :param exclude_frames: Explicit frame pairs excluded by geometry resolution,
        e.g. a camera and its mounting link, using qualified ``instance/frame``.
    :param margin: Extra surface clearance in metres, nonnegative.
    :param alpha1: Positive first barrier gain in s^-1.
    :param alpha2: Positive second barrier gain in s^-1.
    :param residual_tolerance: Positive runtime acceptance tolerance; see
        :class:`robo_arch.core.controllers.cbf.definition.CbfParameters` for units.

    Invalid fields, URIs or protected selections raise ValidationError. Resource
    existence, exclusions and runtime frame identities are checked later. No
    robot, nominal controller or task is selected by these parameters.
    """

    backend: Literal["drake", "torch_moreau"] = "drake"
    compile_model: bool = False
    profiles: dict[str, str]
    protected: tuple[str, ...]
    exclude_frames: tuple[tuple[str, str], ...] = ()
    margin: float = Field(default=0.01, ge=0)
    alpha1: float = Field(default=5.0, gt=0)
    alpha2: float = Field(default=5.0, gt=0)
    velocity_limit_gain: float = Field(default=20.0, gt=0)
    residual_tolerance: float = Field(default=1e-6, gt=0)

    @model_validator(mode="after")
    def references(self) -> Self:
        if self.compile_model and self.backend != "torch_moreau":
            raise ValueError("compile_model requires backend: torch_moreau")
        for resource in self.profiles.values():
            validate_package_reference(resource)
        if not self.protected or len(set(self.protected)) != len(self.protected):
            raise ValueError("Protected instances must be nonempty and unique")
        if not set(self.protected) <= self.profiles.keys():
            raise ValueError("Every protected instance needs a sphere profile")
        return self
