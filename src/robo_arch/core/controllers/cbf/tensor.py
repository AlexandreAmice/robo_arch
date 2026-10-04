"""Batched sphere CBF using shared equations and device-resident tensor dynamics."""

from collections.abc import Callable
from dataclasses import replace
from typing import Any

import torch

from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry
from robo_arch.core.controllers.cbf.barrier import (
    BarrierConstraints,
    geometry_constraints,
)
from robo_arch.core.controllers.cbf.definition import CbfParameters
from robo_arch.core.controllers.cbf.errors import CbfFailure
from robo_arch.core.controllers.cbf.filter import (
    FilterResult as TensorFilterResult,
)
from robo_arch.core.controllers.cbf.filter import (
    project,
    validate_initial_state,
)
from robo_arch.core.controllers.cbf.layout import compile_geometry
from robo_arch.core.controllers.cbf.velocity import compile_velocity_bounds
from robo_arch.core.controllers.dynamics.torch import ModelEvaluation, TensorModel


class TensorCbfFilter:
    """One model and one batched QP workspace; no loop over environments.

    Geometry/configuration and barrier equations are shared with the Drake path.
    The numerical model and projection are injected to allow CPU tensor parity
    tests. The production projection requires CUDA. Failure stops the entire
    batch before commands can be applied; it never supplies nominal fallback.
    """

    def __init__(
        self,
        *,
        model: TensorModel,
        geometry: ProtectionGeometry,
        projection: Any,
        parameters: CbfParameters | None = None,
        command_dtype: torch.dtype | None = None,
    ):
        self.model = model
        self.projection = projection
        self.parameters = parameters or CbfParameters()
        self.command_dtype = command_dtype
        self.count = model.count
        self.geometry = geometry
        layout = compile_geometry(
            geometry.spheres, geometry.pairs, geometry.planes, geometry.plane_pairs
        )
        self.pair_names = layout.pair_names
        self.constraint_count = len(self.pair_names)
        self.diagnostics_size = 5 * self.constraint_count + 3
        self.device, self.dtype = model.device, model.dtype
        bounds = compile_velocity_bounds(
            model.joints, model.velocity_lower, model.velocity_upper
        )
        self._velocity_bounds = replace(
            bounds,
            indices=torch.as_tensor(bounds.indices, device=self.device),
            signs=torch.as_tensor(bounds.signs, device=self.device),
            offsets=torch.as_tensor(bounds.offsets, device=self.device),
        )
        self.velocity_bound_names = self._velocity_bounds.names
        self.qp_constraint_count = self.constraint_count + len(bounds.names)
        self._layout = replace(
            layout,
            **{
                name: torch.as_tensor(
                    getattr(layout, name),
                    device=self.device,
                    dtype=torch.long
                    if name in {"first", "second", "plane_spheres"}
                    else self.dtype,
                )
                for name in (
                    "first",
                    "second",
                    "separation",
                    "local_centers",
                    "plane_spheres",
                    "plane_normals",
                    "plane_offsets",
                )
            },
        )

    def _state(self, state: torch.Tensor) -> None:
        if (
            state.ndim != 2
            or state.shape[1] != 2 * self.count
            or state.device != self.device
            or state.dtype != self.dtype
        ):
            raise ValueError(
                "CBF state needs batched ordered [q, v] on the model device/dtype"
            )
        if not torch.isfinite(state).all():
            raise CbfFailure("nonfinite batched state")

    def evaluate(
        self, state: torch.Tensor, *, evaluation: ModelEvaluation | None = None
    ) -> BarrierConstraints[torch.Tensor]:
        """Return shared barrier rows; optionally reuse current nominal dynamics."""
        self._state(state)
        data = self.model.evaluate(state) if evaluation is None else evaluation
        return self._constraints(state, data)

    def _constraints(
        self, state: torch.Tensor, data: ModelEvaluation
    ) -> BarrierConstraints[torch.Tensor]:
        """Build rows from an already validated state and its current dynamics."""
        if not data.valid.all():
            raise CbfFailure(
                "invalid batched nominal dynamics",
                environments=(~data.valid).nonzero().flatten().cpu().tolist(),
            )
        return geometry_constraints(
            positions=data.positions,
            jacobians=data.jacobians,
            bias_accelerations=data.bias_accelerations,
            velocity=state[:, self.count :],
            acceleration_drift=data.acceleration_drift,
            acceleration_control=data.acceleration_control,
            parameters=self.parameters,
            layout=self._layout,
            namespace=torch,
        )

    def validate_initial_state(self, state: torch.Tensor) -> None:
        rows = self.evaluate(state)
        validate_initial_state(
            rows,
            self._velocity_bounds.slack(state[:, self.count :]),
            pair_names=self.pair_names,
            velocity_names=self.velocity_bound_names,
            namespace=torch,
        )

    @torch.no_grad()
    def filter(
        self,
        state: torch.Tensor,
        nominal_effort: torch.Tensor,
        *,
        evaluation: ModelEvaluation | None = None,
    ) -> TensorFilterResult:
        """Project all commands with one batched solve and hard residual checks."""
        if (
            nominal_effort.shape != (state.shape[0], self.count)
            or nominal_effort.device != self.device
            or nominal_effort.dtype != self.dtype
        ):
            raise ValueError("Nominal effort must match the model batch/device/dtype")
        self._state(state)
        evaluation = self.model.evaluate(state) if evaluation is None else evaluation
        rows = self._constraints(state, evaluation)
        velocity_coefficient, velocity_constant = self._velocity_bounds.constraints(
            state[:, self.count :],
            evaluation.acceleration_drift,
            evaluation.acceleration_control,
            self.parameters.velocity_limit_gain,
        )
        return project(
            rows=rows,
            velocity_coefficient=velocity_coefficient,
            velocity_constant=velocity_constant,
            velocity_slack=self._velocity_bounds.slack(state[:, self.count :]),
            nominal=nominal_effort,
            limits=self.model.limits,
            solve=self.projection.solve,
            tolerance=self.parameters.residual_tolerance,
            namespace=torch,
            rounding_error=(
                None
                if self.command_dtype is None
                else torch.finfo(self.command_dtype).eps * self.model.limits
            ),
            round_command=(
                None
                if self.command_dtype is None
                else lambda effort: effort.to(self.command_dtype).to(self.dtype)
            ),
        )

    def wrap(
        self, nominal: Callable[[torch.Tensor, Any], torch.Tensor]
    ) -> Callable[[torch.Tensor, Any], torch.Tensor]:
        """Wrap an arbitrary compatible batched nominal effort callable."""

        def command(state: torch.Tensor, time: Any) -> torch.Tensor:
            return self.filter(state, nominal(state, time)).effort

        return command
