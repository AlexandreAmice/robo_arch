"""Shared scalar/batched CBF acceptance and diagnostics, without simulator SDKs."""

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

import numpy as np

from robo_arch.core.controllers.cbf.barrier import BarrierConstraints
from robo_arch.core.controllers.cbf.errors import CbfFailure


@dataclass(frozen=True)
class FilterResult[Array]:
    """Owned commands and geometry diagnostics, with optional velocity diagnostics.

    Leading axes represent independent environments. Timing is NaN when the
    numerical provider does not measure individual solves. Active flags describe
    the enforced QP boundary; residuals describe the delivered command.
    """

    effort: Array
    diagnostics: Array
    velocity_slack: Array | None = None
    velocity_residual: Array | None = None


def validate_initial_state(
    rows: BarrierConstraints,
    velocity_slack: Any,
    *,
    pair_names: tuple[str, ...],
    velocity_names: tuple[str, ...],
    namespace: Any = np,
    snapshot: Callable[[], dict] = dict,
) -> None:
    """Enforce the same geometric and velocity domain for every provider."""
    xp = namespace
    invalid_velocity = ~xp.isfinite(velocity_slack) | (velocity_slack < -1e-10)
    if invalid_velocity.any():
        raise CbfFailure(
            "inadmissible initial velocity",
            **snapshot(),
            environment_bound_indices=xp.argwhere(invalid_velocity).tolist(),
            bounds=velocity_names,
        )
    valid = xp.isfinite(rows.h) & xp.isfinite(rows.psi1)
    valid &= (rows.h >= -1e-10) & (rows.psi1 >= -1e-10)
    if not valid.all():
        raise CbfFailure(
            "inadmissible initial state",
            **snapshot(),
            environment_constraint_indices=xp.argwhere(~valid).tolist(),
            pairs=pair_names,
            h=rows.h.tolist(),
            psi1=rows.psi1.tolist(),
        )


def project[Array](
    *,
    rows: BarrierConstraints[Array],
    velocity_coefficient: Array,
    velocity_constant: Array,
    velocity_slack: Array,
    nominal: Array,
    limits: Array,
    solve: Callable[[Array, Array, Array], Array],
    tolerance: float,
    namespace: Any = np,
    rounding_error: Array | None = None,
    round_command: Callable[[Array], Array] | None = None,
    solve_duration: Callable[[], float] = lambda: float("nan"),
    snapshot: Callable[[], dict] = dict,
) -> FilterResult[Array]:
    """Assemble, project, validate and report one scalar or batched effort command.

    Providers supply numerical dynamics upstream and a bounded QP solver here.
    Native command precision can tighten rows before projection, but acceptance
    always checks the delivered command against the original constraints.
    Only failure snapshots transfer array values to host Python objects.
    """
    xp = namespace
    coefficient = xp.concatenate((rows.coefficient, velocity_coefficient), axis=-2)
    original_constant = xp.concatenate((rows.constant, velocity_constant), axis=-1)
    constant = original_constant
    if rounding_error is not None:
        constant = constant - xp.einsum(
            "...pi,i->...p", xp.abs(coefficient), rounding_error
        )
    effort = solve(coefficient, constant, nominal)
    projected_residual = xp.einsum("...pi,...i->...p", coefficient, effort) + constant
    if round_command is not None:
        effort = round_command(effort)
        residual = (
            xp.einsum("...pi,...i->...p", coefficient, effort) + original_constant
        )
    else:
        residual = projected_residual
    valid = (
        xp.isfinite(effort).all(axis=-1)
        & xp.isfinite(residual).all(axis=-1)
        & (residual >= -tolerance).all(axis=-1)
        & (xp.abs(effort) <= limits + tolerance).all(axis=-1)
    )
    if not valid.all():
        raise CbfFailure(
            "QP residual check failed",
            **snapshot(),
            environments=xp.argwhere(~valid).reshape(-1).tolist(),
            effort=effort.tolist(),
            residual=residual.tolist(),
        )
    count = rows.constant.shape[-1]
    correction = xp.sqrt(((effort - nominal) ** 2).sum(axis=-1, keepdims=True))
    diagnostics = xp.concatenate(
        (
            rows.clearance,
            rows.h,
            rows.psi1,
            residual[..., :count],
            (projected_residual[..., :count] <= 10 * tolerance) * 1.0,
            correction,
            xp.full_like(correction, solve_duration()),
            xp.ones_like(correction),
        ),
        axis=-1,
    )
    return FilterResult(
        effort * 1.0, diagnostics, velocity_slack * 1.0, residual[..., count:] * 1.0
    )
