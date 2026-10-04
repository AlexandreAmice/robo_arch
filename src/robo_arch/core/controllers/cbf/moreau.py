"""CUDA batched bounded effort projection through Moreau's Torch interface.

Numerical problem data and solutions stay on CUDA. Moreau returns host status
metadata; Python validation also synchronizes scalar predicates before accepting
a command. This is not a synchronization-free or differentiable controller.
"""

import math

import moreau
import torch
from moreau.torch import Solver

from robo_arch.core.controllers.cbf.errors import CbfFailure


class MoreauProjection:
    """Minimize 0.5 ||u-nominal||² subject to A u+b>=0 and |u|<=limits.

    One instance owns one fixed-size batched solver; calls must be sequential.
    Inputs use CUDA float64, shape (batch, rows, count), (batch, rows), and
    (batch, count). No warm-start conversion or CPU solver fallback is used.
    """

    def __init__(
        self,
        *,
        count: int,
        constraint_count: int,
        batch_size: int,
        limits: torch.Tensor,
        device: torch.device | str,
        residual_tolerance: float = 1e-6,
    ) -> None:
        self.device = torch.device(device)
        if self.device.type != "cuda":
            raise ValueError("MoreauProjection requires a CUDA device")
        if self.device.index is None:
            self.device = torch.device("cuda", torch.cuda.current_device())
        if count < 1 or constraint_count < 0 or batch_size < 1:
            raise ValueError("Invalid effort projection dimensions")
        if not math.isfinite(residual_tolerance) or residual_tolerance <= 0:
            raise ValueError("Residual tolerance must be finite and positive")
        self.count = count
        self.constraint_count = constraint_count
        self.batch_size = batch_size
        self.residual_tolerance = residual_tolerance
        self._check_tensor(limits, (count,), "limits")
        if not bool((torch.isfinite(limits) & (limits > 0)).all()):
            raise ValueError("Effort limits must be finite and positive")
        self.limits = limits.detach().clone()
        rows = constraint_count + 2 * count
        # CSR topology is constructed once; dense rows retain a fixed structure
        # even when individual barrier coefficients become exactly zero.
        self._identity = torch.eye(count, dtype=torch.float64, device=self.device)
        self._p_values = torch.ones(
            (batch_size, count), dtype=torch.float64, device=self.device
        )
        self._bounds = torch.cat((self._identity, -self._identity)).expand(
            batch_size, -1, -1
        )
        self._solver = Solver(
            n=count,
            m=rows,
            P_row_offsets=torch.arange(count + 1, dtype=torch.int32),
            P_col_indices=torch.arange(count, dtype=torch.int32),
            A_row_offsets=torch.arange(rows + 1, dtype=torch.int32) * count,
            A_col_indices=torch.arange(count, dtype=torch.int32).repeat(rows),
            cones=moreau.Cones(num_nonneg_cones=rows),
            settings=moreau.Settings(
                device="cuda",
                device_id=self.device.index,
                batch_size=batch_size,
                solver="ipm",
                verbose=False,
                ipm_settings=moreau.IPMSettings(
                    tol_gap_abs=1e-10,
                    tol_gap_rel=1e-10,
                    tol_feas=1e-10,
                ),
            ),
        )
        self.last_status: tuple[str, ...] = ()

    def _check_tensor(
        self, value: torch.Tensor, shape: tuple[int, ...], name: str
    ) -> None:
        if (
            not isinstance(value, torch.Tensor)
            or value.shape != shape
            or value.device != self.device
            or value.dtype != torch.float64
        ):
            raise ValueError(
                f"{name} must have shape {shape}, dtype float64, device {self.device}"
            )

    def _failure(
        self,
        reason: str,
        indices: torch.Tensor | list[int],
        coefficient: torch.Tensor,
        constant: torch.Tensor,
        nominal: torch.Tensor,
        **details: object,
    ) -> CbfFailure:
        """Copy only failed QPs for reproduction; never called on accepted solves."""
        indices = torch.as_tensor(indices, dtype=torch.long, device=self.device)
        return CbfFailure(
            reason,
            environments=indices.detach().cpu().tolist(),
            coefficient=coefficient[indices].detach().cpu().tolist(),
            constant=constant[indices].detach().cpu().tolist(),
            nominal=nominal[indices].detach().cpu().tolist(),
            limits=self.limits.detach().cpu().tolist(),
            residual_tolerance=self.residual_tolerance,
            **details,
        )

    @torch.no_grad()
    def solve(
        self,
        coefficient: torch.Tensor,
        constant: torch.Tensor,
        nominal: torch.Tensor,
    ) -> torch.Tensor:
        """Return owned accepted efforts or raise without issuing any command."""
        batch, rows, count = self.batch_size, self.constraint_count, self.count
        self._check_tensor(coefficient, (batch, rows, count), "coefficient")
        self._check_tensor(constant, (batch, rows), "constant")
        self._check_tensor(nominal, (batch, count), "nominal")
        finite = (
            torch.isfinite(coefficient).all(dim=(1, 2))
            & torch.isfinite(constant).all(dim=1)
            & torch.isfinite(nominal).all(dim=1)
        )
        if not bool(finite.all()):
            raise self._failure(
                "nonfinite Moreau input",
                torch.where(~finite)[0],
                coefficient,
                constant,
                nominal,
            )
        nominal_residual = (coefficient @ nominal.unsqueeze(-1)).squeeze(-1) + constant
        feasible = (nominal_residual >= 0).all(dim=1) & (
            nominal.abs() <= self.limits
        ).all(dim=1)
        if bool(feasible.all()):
            self.last_status = ("NominalFeasible",) * batch
            return nominal.clone()
        scale = torch.maximum(coefficient.abs().amax(dim=2), constant.abs()).clamp_min(
            1.0
        )
        # Moreau uses A_qp u+s=b_qp, s>=0: negate the barrier coefficients.
        matrix = torch.cat((-coefficient / scale.unsqueeze(-1), self._bounds), dim=1)
        rhs = torch.cat(
            (constant / scale, self.limits.expand(batch, -1).repeat(1, 2)), dim=1
        )
        solution = self._solver.solve(
            self._p_values, matrix.flatten(start_dim=1), -nominal, rhs
        )
        status = self._solver.info.status
        status = status if isinstance(status, list) else [status]
        self.last_status = tuple(str(value) for value in status)
        failed = [
            index
            for index, value in enumerate(status)
            if value != moreau.SolverStatus.Solved
        ]
        if failed:
            raise self._failure(
                "Moreau solve failed",
                failed,
                coefficient,
                constant,
                nominal,
                status=self.last_status,
            )
        effort = solution.x.reshape(batch, count)
        # Exact feasible nominal commands already minimize the objective.
        effort = torch.where(feasible[:, None], nominal, effort).clone()
        residual = (coefficient @ effort.unsqueeze(-1)).squeeze(-1) + constant
        accepted = (
            torch.isfinite(effort).all(dim=1)
            & torch.isfinite(residual).all(dim=1)
            & (residual >= -self.residual_tolerance).all(dim=1)
            & (effort.abs() <= self.limits + self.residual_tolerance).all(dim=1)
        )
        if not bool(accepted.all()):
            raise self._failure(
                "Moreau solution violated hard constraints",
                torch.where(~accepted)[0],
                coefficient,
                constant,
                nominal,
                status=self.last_status,
            )
        return effort
