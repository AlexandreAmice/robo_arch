"""Batched fixed-base rigid-body kinematics and dynamics on a Torch device.

Model loaders supply fixed device constants. Evaluation keeps state and outputs
on the selected device and batches environments; no simulator SDK is imported.
"""

from dataclasses import dataclass, fields

import torch


@dataclass(frozen=True)
class ModelEvaluation:
    """Owned batched world-expressed point kinematics and ordered joint dynamics.

    Leading dimension is environment; positions/biases are [B,S,3], Jacobians
    [B,S,3,n], mass/control [B,n,n], and joint forces/drift [B,n]. Bias force
    includes Coriolis, gravity and damping: M vdot = effort - bias_force.
    """

    positions: torch.Tensor
    jacobians: torch.Tensor
    bias_accelerations: torch.Tensor
    mass: torch.Tensor
    bias_force: torch.Tensor
    acceleration_drift: torch.Tensor
    acceleration_control: torch.Tensor
    valid: torch.Tensor


def _mv(rotation: torch.Tensor, vector: torch.Tensor) -> torch.Tensor:
    return (rotation @ vector.unsqueeze(-1)).squeeze(-1)


def _cross(first: torch.Tensor, second: torch.Tensor) -> torch.Tensor:
    return torch.linalg.cross(first, second, dim=-1)


def _point_jacobian(
    linear: torch.Tensor, angular: torch.Tensor, offset: torch.Tensor
) -> torch.Tensor:
    return linear + _cross(angular.transpose(-1, -2), offset[:, None]).transpose(-1, -2)


@dataclass(frozen=True)
class Body:
    """Device constants for one tree body, in parent/joint/body frames."""

    parent: int
    dof: int | None
    kind: str
    rotation_parent: torch.Tensor
    translation_parent: torch.Tensor
    rotation_child: torch.Tensor
    translation_child: torch.Tensor
    axis: torch.Tensor
    axis_cross: torch.Tensor
    mass: float
    com: torch.Tensor
    inertia: torch.Tensor
    gravity: torch.Tensor


class TensorModel:
    """Independent nominal model for scalar revolute/prismatic joints and welds.

    Actuated coordinates have matching q, v and effort order. The numerical
    model supports uniform gravity, joint damping and reflected rotor inertia.
    SDK-specific loaders validate source models before constructing constants.
    """

    def __init__(
        self,
        *,
        joints: tuple[str, ...],
        velocity_lower: tuple[float, ...],
        velocity_upper: tuple[float, ...],
        limits: torch.Tensor,
        damping: torch.Tensor,
        rotor: torch.Tensor,
        bodies: tuple[Body, ...],
        point_bodies: torch.Tensor,
        points: torch.Tensor,
    ) -> None:
        """Take ownership of fixed device constants produced by a model loader.

        Floating constants must be float64 on ``limits.device``; indices use
        int64. Callers relinquish mutation of these arrays. Body order starts
        after the implicit world body and every parent precedes its children.
        """
        self.count = len(joints)
        self.joints = joints
        self.velocity_lower = velocity_lower
        self.velocity_upper = velocity_upper
        self.device = limits.device
        self.dtype = limits.dtype
        self.limits = limits
        self._damping = damping
        self._rotor = rotor
        self._bodies = bodies
        self._point_bodies = point_bodies
        self._points = points
        self._identity = torch.eye(self.count, device=self.device, dtype=self.dtype)
        self._identity3 = torch.eye(3, device=self.device, dtype=self.dtype)
        self._compiled_evaluate = None

    def enable_compilation(self) -> None:
        """Opt into lazy full-graph Torch compilation of the same equations.

        First evaluation for a new shape includes compilation and CUDA graph
        warmup. Public outputs are cloned outside the compiled graph so later
        evaluations cannot overwrite values retained by callers.
        """
        if self._compiled_evaluate is None:
            self._compiled_evaluate = torch.compile(
                self._evaluate, fullgraph=True, mode="reduce-overhead"
            )

    def evaluate(self, state: torch.Tensor) -> ModelEvaluation:
        """Evaluate [B,2n] ordered q/v without simulator calls or host transfers."""
        if state.ndim != 2 or state.shape[1] != 2 * self.count:
            raise ValueError("Tensor dynamics state must have shape [batch, 2*joints]")
        if state.dtype != self.dtype or state.device != self.device:
            raise ValueError("Tensor dynamics state dtype/device must match its model")
        if self._compiled_evaluate is None:
            return self._evaluate(state)
        result = self._compiled_evaluate(state)
        return ModelEvaluation(
            **{
                field.name: getattr(result, field.name).clone()
                for field in fields(result)
            }
        )

    def _evaluate(self, state: torch.Tensor) -> ModelEvaluation:
        batch, n = state.shape[0], self.count
        q, v = state[:, :n], state[:, n:]
        zeros = state.new_zeros((batch, 3))
        zero_j = state.new_zeros((batch, 3, n))
        rotations = [self._identity3.expand(batch, 3, 3)]
        positions, omegas, velocities = [zeros], [zeros], [zeros]
        alphas, accelerations, angular_j, linear_j = (
            [zeros],
            [zeros],
            [zero_j],
            [zero_j],
        )
        mass = torch.diag_embed(self._rotor.expand(batch, n))
        bias = self._damping * v
        for body in self._bodies:
            p = body.parent
            rotation = rotations[p] @ body.rotation_parent
            offset = _mv(rotations[p], body.translation_parent)
            position = positions[p] + offset
            omega, alpha = omegas[p], alphas[p]
            velocity = velocities[p] + _cross(omega, offset)
            acceleration = (
                accelerations[p]
                + _cross(alpha, offset)
                + _cross(omega, _cross(omega, offset))
            )
            jw = angular_j[p]
            jv = _point_jacobian(linear_j[p], jw, offset)
            if body.dof is not None:
                index = body.dof
                angle, speed = q[:, index], v[:, index]
                axis = _mv(rotation, body.axis)
                column = self._identity[index]
                if body.kind == "revolute":
                    k = body.axis_cross
                    turn = (
                        self._identity3
                        + torch.sin(angle)[:, None, None] * k
                        + (1 - torch.cos(angle))[:, None, None] * (k @ k)
                    )
                    rotation = rotation @ turn
                    alpha = alpha + _cross(omega, axis) * speed[:, None]
                    omega = omega + axis * speed[:, None]
                    jw = jw + axis[:, :, None] * column
                else:
                    offset = axis * angle[:, None]
                    position = position + offset
                    acceleration = (
                        acceleration
                        + _cross(alpha, offset)
                        + _cross(omega, _cross(omega, offset))
                        + 2 * _cross(omega, axis) * speed[:, None]
                    )
                    velocity = velocity + _cross(omega, offset) + axis * speed[:, None]
                    jv = _point_jacobian(jv, jw, offset) + axis[:, :, None] * column
            offset = _mv(rotation, body.translation_child)
            position = position + offset
            velocity = velocity + _cross(omega, offset)
            acceleration = (
                acceleration
                + _cross(alpha, offset)
                + _cross(omega, _cross(omega, offset))
            )
            jv = _point_jacobian(jv, jw, offset)
            rotation = rotation @ body.rotation_child
            rotations.append(rotation)
            positions.append(position)
            omegas.append(omega)
            velocities.append(velocity)
            alphas.append(alpha)
            accelerations.append(acceleration)
            angular_j.append(jw)
            linear_j.append(jv)
            com_offset = _mv(rotation, body.com)
            com_j = _point_jacobian(jv, jw, com_offset)
            com_acceleration = (
                acceleration
                + _cross(alpha, com_offset)
                + _cross(omega, _cross(omega, com_offset))
            )
            inertia = rotation @ body.inertia @ rotation.transpose(-1, -2)
            mass = (
                mass
                + body.mass * (com_j.transpose(-1, -2) @ com_j)
                + jw.transpose(-1, -2) @ inertia @ jw
            )
            force = body.mass * (com_acceleration - body.gravity)
            torque = _mv(inertia, alpha) + _cross(omega, _mv(inertia, omega))
            bias = (
                bias
                + _mv(com_j.transpose(-1, -2), force)
                + _mv(jw.transpose(-1, -2), torque)
            )
        indices = self._point_bodies
        point_rotation = torch.stack(rotations, dim=1)[:, indices]
        offset = _mv(point_rotation, self._points)
        point_positions = torch.stack(positions, dim=1)[:, indices] + offset
        point_jw = torch.stack(angular_j, dim=1)[:, indices]
        point_jv = torch.stack(linear_j, dim=1)[:, indices]
        point_jv = point_jv + _cross(
            point_jw.transpose(-1, -2), offset[:, :, None]
        ).transpose(-1, -2)
        point_omega = torch.stack(omegas, dim=1)[:, indices]
        point_alpha = torch.stack(alphas, dim=1)[:, indices]
        point_bias = (
            torch.stack(accelerations, dim=1)[:, indices]
            + _cross(point_alpha, offset)
            + _cross(point_omega, _cross(point_omega, offset))
        )
        rhs = torch.cat((-bias[:, :, None], self._identity.expand(batch, n, n)), dim=-1)
        acceleration, info = torch.linalg.solve_ex(mass, rhs, check_errors=False)
        valid = (
            (info == 0)
            & torch.isfinite(state).all(dim=-1)
            & torch.isfinite(acceleration).all(dim=(-2, -1))
        )
        return ModelEvaluation(
            point_positions,
            point_jv,
            point_bias,
            mass,
            bias,
            acceleration[:, :, 0],
            acceleration[:, :, 1:],
            valid,
        )
