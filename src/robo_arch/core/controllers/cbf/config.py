"""SDK-independent selection and tuning for sphere-based protection."""

from typing import Self

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.resources import validate_package_reference


class ProtectionParameters(Parameters):
    """Protection geometry and barrier tuning, independent of nominal control."""

    profiles: dict[str, str]
    protected: tuple[str, ...]
    exclude_frames: tuple[tuple[str, str], ...] = ()
    margin: float = Field(default=0.01, ge=0)
    alpha1: float = Field(default=5.0, gt=0)
    alpha2: float = Field(default=5.0, gt=0)
    residual_tolerance: float = Field(default=1e-6, gt=0)

    @model_validator(mode="after")
    def references(self) -> Self:
        for resource in self.profiles.values():
            validate_package_reference(resource)
        if not self.protected or len(set(self.protected)) != len(self.protected):
            raise ValueError("Protected instances must be nonempty and unique")
        if not set(self.protected) <= self.profiles.keys():
            raise ValueError("Every protected instance needs a sphere profile")
        return self
