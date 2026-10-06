"""SDK-independent Isaac world settings."""

from typing import Literal

from pydantic import Field

from robo_arch.core.config.schema import Schema


class IsaacPhysics(Schema):
    """PhysX scene settings, declared independently of the Isaac SDK.

    :param backend: Fixed discriminator ``physx``.
    :param time_step: Positive physics step in seconds (default 0.001).
    :param solver: Temporal Gauss-Seidel (tgs) or projected Gauss-Seidel (pgs).
    :param device: cpu or cuda:0 physics selection. This does not move autonomy
        code onto the GPU or guarantee that the selected combination can run.
    :param gpu_found_lost_aggregate_pairs_capacity: Positive integer broadphase
        aggregate-pair capacity; unset uses the native SDK default.

    Unknown/invalid fields raise ValidationError; runtime compatibility requires
    the separately pinned Isaac environment.
    """

    backend: Literal["physx"] = "physx"
    time_step: float = Field(default=0.001, gt=0)
    solver: Literal["tgs", "pgs"] = "tgs"
    device: Literal["cpu", "cuda:0"] = "cuda:0"
    gpu_found_lost_aggregate_pairs_capacity: int | None = Field(default=None, gt=0)


class NewtonPhysics(Schema):
    """Newton's GPU MuJoCo Warp solver, declared without importing the SDK.

    :param backend: Required discriminator ``newton``.
    :param solver: Fixed solver ``mujoco_warp``.
    :param time_step: Positive physics step in seconds (default 0.001).
    :param device: cuda:0; CPU execution is unsupported in this profile.
    :param iterations: Positive maximum constraint solver iterations.
    :param ls_iterations: Positive maximum line search iterations.
    :param integrator: euler, implicitfast or rk4 native integration.
    :param constraint_solver: newton or cg native constraint solver.
    :param njmax: Positive constraint capacity per environment.
    :param nconmax: Optional positive contact capacity per environment.

    Invalid fields raise ValidationError. Sensor observations are unsupported;
    disabling observations retains the mounted sensor bodies.
    """

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
    """Native Isaac Storm viewer settings; no viewer is created by this model.

    :param type: Fixed discriminator ``isaac``.
    :param mode: off or live; recording/RTX camera modes are not supported here.
    :param publish_period: Positive simulation-time display interval in seconds.
    :param width: Positive viewport width in pixels.
    :param height: Positive viewport height in pixels.

    Invalid fields raise ValidationError. Closing and holding the viewer belong
    to the runner; the selected mode does not alter physical geometry.
    """

    type: Literal["isaac"] = "isaac"
    mode: Literal["off", "live"] = "off"
    publish_period: float = Field(default=1 / 30, gt=0)
    width: int = Field(default=960, gt=0)
    height: int = Field(default=720, gt=0)


class IsaacWorld(Schema):
    """Complete Isaac world profile, inspectable without importing Isaac.

    :param type: Fixed discriminator ``isaac``.
    :param ground: Static collision plane at world z=0 with a finite visual.
    :param target_realtime_rate: Nonnegative simulated/wall-time pacing ratio
        while viewing; zero is unpaced. Headless stepping does not wait.
    :param num_envs: Number of independent physical environments (default one).
    :param env_layout: line or grid placement of environment origins.
    :param env_spacing: Positive separation between origins, in metres.
    :param physics: IsaacPhysics or NewtonPhysics settings; defaults to PhysX.
    :param visualization: IsaacVisualization settings, off by default.

    Invalid fields raise ValidationError. Execution still requires the vendor
    environment and compatible assets/devices; a valid profile alone is not proof.
    """

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
