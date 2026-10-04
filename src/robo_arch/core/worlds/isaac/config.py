"""SDK-independent Isaac world settings."""

from typing import Literal

from pydantic import Field, model_validator

from robo_arch.core.config.schema import Schema


class IsaacPhysics(Schema):
    """Native PhysX scene settings; the time step is in seconds."""

    time_step: float = Field(default=0.001, gt=0)
    solver: Literal["tgs", "pgs"] = "tgs"
    device: Literal["cpu", "cuda:0"] = "cuda:0"
    # Needed by the explicit convex sectors in mounted Mini45 collision geometry.
    gpu_found_lost_aggregate_pairs_capacity: int = Field(default=32768, gt=0)


class IsaacVisualization(Schema):
    type: Literal["isaac"] = "isaac"
    mode: Literal["off", "live"] = "off"
    publish_period: float = Field(default=1 / 30, gt=0)
    width: int = Field(default=960, gt=0)
    height: int = Field(default=720, gt=0)


class IsaacWorld(Schema):
    type: Literal["isaac"] = "isaac"
    # Static collision plane at world z=0, with a finite 10 m square visual.
    ground: bool = True
    control_backend: Literal["numpy", "torch"] = "numpy"
    batch_size: int = Field(default=1, gt=0)
    environment_spacing: float = Field(default=3.0, gt=0)
    log_every_n_steps: int = Field(default=1, gt=0)
    # Pacing applies only when viewing; headless physics runs without wall-clock waits.
    target_realtime_rate: float = Field(default=1.0, ge=0)
    physics: IsaacPhysics = Field(default_factory=IsaacPhysics)
    visualization: IsaacVisualization = Field(default_factory=IsaacVisualization)

    @model_validator(mode="after")
    def validate_control_backend(self) -> "IsaacWorld":
        if self.control_backend == "torch" and self.physics.device != "cuda:0":
            raise ValueError("Torch control requires native CUDA physics on cuda:0")
        if self.control_backend == "numpy" and self.batch_size != 1:
            raise ValueError("Batched Isaac execution requires torch control")
        return self
