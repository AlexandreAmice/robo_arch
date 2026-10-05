"""SDK-independent task and measurement settings."""

from typing import Literal

from pydantic import Field, model_validator

from robo_arch.core.config.schema import Schema


class Reaching(Schema):
    seed: int = Field(default=7, ge=0)
    target_radius: float = Field(default=0.15, gt=0)
    tolerance: float = Field(default=0.02, gt=0)
    velocity_tolerance: float = Field(default=0.05, gt=0)
    settle_seconds: float = Field(default=0.1, gt=0)
    episode_seconds: float = Field(default=2.0, gt=0)
    feedforward: Literal["nominal_gravity"] = "nominal_gravity"

    @model_validator(mode="after")
    def episode_can_settle(self) -> "Reaching":
        if self.settle_seconds > self.episode_seconds:
            raise ValueError("Settle duration must not exceed episode duration")
        return self


class Measurement(Schema):
    warmup_steps: int = Field(default=200, ge=0)
    sample_period: float = Field(default=0.02, gt=0)
    sampled_envs: int = Field(default=4, ge=0)
    reset_period: float = Field(default=0.02, gt=0)
    controller_mode: Literal["tensor", "scalar"] = "tensor"
