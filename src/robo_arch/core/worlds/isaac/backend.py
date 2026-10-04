"""Small adapters for the two supported Isaac Lab physics backends."""

from typing import Any

from robo_arch.core.worlds.isaac.config import IsaacPhysics, NewtonPhysics


def validate_observations(
    config: IsaacPhysics | NewtonPhysics, names: tuple[str, ...]
) -> None:
    """Reject unsupported observation requests before constructing native sensors.

    The pinned Newton wrench sensor excludes FIXED joints and cannot represent
    the Mini45 sensing joint. Physical mounted bodies remain supported.
    """
    if isinstance(config, NewtonPhysics) and names:
        raise ValueError(
            "Newton sensor observations are unsupported in this profile: "
            f"{', '.join(names)}. Use sensors_enabled: false (--no-sensors) to "
            "retain physical sensor models without observations."
        )


def physics_config(config: IsaacPhysics | NewtonPhysics) -> Any:
    """Import only the selected backend after application startup."""
    if isinstance(config, IsaacPhysics):
        from isaaclab_physx.physics import PhysxCfg

        return PhysxCfg(
            solver_type=1 if config.solver == "tgs" else 0,
            gpu_found_lost_aggregate_pairs_capacity=(
                config.gpu_found_lost_aggregate_pairs_capacity
            ),
        )
    from isaaclab_newton.physics import MJWarpSolverCfg, NewtonCfg

    return NewtonCfg(
        solver_cfg=MJWarpSolverCfg(
            iterations=config.iterations,
            ls_iterations=config.ls_iterations,
            integrator=config.integrator,
            solver=config.constraint_solver,
            njmax=config.njmax,
            nconmax=config.nconmax,
        )
    )


def cloning_contexts(config: IsaacPhysics | NewtonPhysics) -> tuple:
    from isaaclab.cloner import UsdReplicateContext

    if isinstance(config, IsaacPhysics):
        return (UsdReplicateContext,)
    from isaaclab_newton.cloner.replicate import NewtonReplicateContext

    return (NewtonReplicateContext, UsdReplicateContext)
