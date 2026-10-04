"""SDK-independent Isaac world settings."""

from typing import Literal

from pydantic import Field

from robo_arch.core.config.schema import Schema


class IsaacPhysics(Schema):
    """Native PhysX scene settings; the time step is in seconds."""

    backend: Literal["physx"] = "physx"
    time_step: float = Field(default=0.001, gt=0)
    solver: Literal["tgs", "pgs"] = "tgs"
    device: Literal["cpu", "cuda:0"] = "cuda:0"
    # Needed by the explicit convex sectors in mounted Mini45 collision geometry.
    gpu_found_lost_aggregate_pairs_capacity: int = Field(default=32768, gt=0)


class NewtonPhysics(Schema):
    """Newton's GPU MuJoCo Warp solver; time step is in seconds."""

    backend: Literal["newton"]
    solver: Literal["mujoco_warp"] = "mujoco_warp"
    time_step: float = Field(default=0.001, gt=0)
    device: Literal["cuda:0"] = "cuda:0"
    iterations: int = Field(default=100, ge=1)
    ls_iterations: int = Field(default=50, ge=1)
    integrator: Literal["euler", "implicitfast", "rk4"] = "euler"
    constraint_solver: Literal["newton", "cg"] = "newton"
    njmax: int = Field(default=300, ge=1)
    nconmax: int | None = Field(default=None, ge=1)


class IsaacVisualization(Schema):
    type: Literal["isaac"] = "isaac"
    mode: Literal["off", "live"] = "off"
    publish_period: float = Field(default=1 / 30, gt=0)
    width: int = Field(default=960, gt=0)
    height: int = Field(default=720, gt=0)


class IsaacWorld(Schema):
    type: Literal["isaac"] = "isaac"
    num_envs: int = Field(default=1, ge=1)
    env_layout: Literal["line", "grid"] = "line"
    env_spacing: float = Field(default=3.0, gt=0)
    # Static collision plane at world z=0, with a finite 10 m square visual.
    ground: bool = True
    # Pacing applies only when viewing; headless physics runs without wall-clock waits.
    target_realtime_rate: float = Field(default=1.0, ge=0)
    physics: IsaacPhysics | NewtonPhysics = Field(default_factory=IsaacPhysics)
    visualization: IsaacVisualization = Field(default_factory=IsaacVisualization)
