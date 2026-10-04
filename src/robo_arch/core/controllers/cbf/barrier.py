"""Second-order sphere separation constraints, independent of a physics SDK."""

from dataclasses import dataclass
from typing import Any

import numpy as np

from robo_arch.core.controllers.cbf.definition import CbfParameters
from robo_arch.core.controllers.cbf.layout import ConstraintLayout


@dataclass(frozen=True)
class BarrierConstraint:
    """Affine inequality ``coefficient @ effort + constant >= 0``."""

    coefficient: np.ndarray
    constant: float
    clearance: float
    h: float
    psi1: float


@dataclass(frozen=True)
class BarrierConstraints[Array]:
    """One row per protection pair, with all numerical fields stored as arrays."""

    coefficient: Array
    constant: Array
    clearance: Array
    h: Array
    psi1: Array

    def rows(self) -> tuple[BarrierConstraint, ...]:
        """Return owned scalar rows for inspecting individual constraints."""
        return tuple(
            BarrierConstraint(a.copy(), float(b), float(c), float(h), float(psi))
            for a, b, c, h, psi in zip(
                self.coefficient,
                self.constant,
                self.clearance,
                self.h,
                self.psi1,
                strict=True,
            )
        )


def sphere_constraints[Array](
    *,
    displacement: Array,
    relative_jacobian: Array,
    relative_bias_acceleration: Array,
    velocity: Array,
    acceleration_drift: Array,
    acceleration_control: Array,
    separation: Array,
    parameters: CbfParameters,
    namespace: Any = np,
) -> BarrierConstraints[Array]:
    """Assemble independent pair rows together, with pair as the leading axis.

    Positions/biases have shape (pairs, 3), Jacobians (pairs, 3, velocities),
    and separation (pairs,). Optional leading axes batch independent models.
    NumPy and Torch execute these same equations; pass their module as namespace.
    Dynamics are shared by every pair in each model.
    """
    d, j = displacement, relative_jacobian
    center_velocity = namespace.einsum("...pij,...j->...pi", j, velocity)
    distance_squared = namespace.einsum("...pi,...pi->...p", d, d)
    h = distance_squared - separation**2
    two_d_j = 2 * namespace.einsum("...pi,...pij->...pj", d, j)
    hdot = namespace.einsum("...pi,...i->...p", two_d_j, velocity)
    acceleration = relative_bias_acceleration + namespace.einsum(
        "...pij,...j->...pi", j, acceleration_drift
    )
    constant = (
        2 * namespace.einsum("...pi,...pi->...p", center_velocity, center_velocity)
        + 2 * namespace.einsum("...pi,...pi->...p", d, acceleration)
        + (parameters.alpha1 + parameters.alpha2) * hdot
        + parameters.alpha1 * parameters.alpha2 * h
    )
    return BarrierConstraints(
        coefficient=namespace.einsum(
            "...pi,...ij->...pj", two_d_j, acceleration_control
        ),
        constant=constant,
        clearance=namespace.sqrt(distance_squared) - separation,
        h=h,
        psi1=hdot + parameters.alpha1 * h,
    )


def sphere_constraint(
    *,
    displacement: np.ndarray,
    relative_jacobian: np.ndarray,
    relative_bias_acceleration: np.ndarray,
    velocity: np.ndarray,
    acceleration_drift: np.ndarray,
    acceleration_control: np.ndarray,
    separation: float,
    parameters: CbfParameters,
) -> BarrierConstraint:
    """Use world-expressed relative kinematics and ``vdot = drift + control*u``.

    The Jacobian maps generalized velocity to center velocity, and bias is
    its time derivative times velocity. Both moving centers contribute.
    """
    return sphere_constraints(
        displacement=np.asarray(displacement)[None, :],
        relative_jacobian=np.asarray(relative_jacobian)[None, :, :],
        relative_bias_acceleration=np.asarray(relative_bias_acceleration)[None, :],
        velocity=velocity,
        acceleration_drift=acceleration_drift,
        acceleration_control=acceleration_control,
        separation=np.array([separation]),
        parameters=parameters,
    ).rows()[0]


def plane_constraints[Array](
    *,
    positions: Array,
    jacobians: Array,
    bias_accelerations: Array,
    normals: Array,
    offsets_with_radius: Array,
    velocity: Array,
    acceleration_drift: Array,
    acceleration_control: Array,
    parameters: CbfParameters,
    namespace: Any = np,
) -> BarrierConstraints[Array]:
    """Assemble sphere-to-fixed-plane rows using linear signed distance.

    All positions, normals and biases are world expressed, with pair preceding
    spatial axes. Optional leading axes batch independent models. NumPy and
    Torch execute the same equations via the namespace argument.
    Offsets include plane offset, sphere radius and extra margin. Unit normals
    make h a distance in meters, unlike squared-distance sphere-pair barriers.
    """
    h = namespace.einsum("...pi,...pi->...p", normals, positions) - offsets_with_radius
    normal_jacobian = namespace.einsum("...pi,...pij->...pj", normals, jacobians)
    hdot = namespace.einsum("...pi,...i->...p", normal_jacobian, velocity)
    constant = (
        namespace.einsum("...pi,...pi->...p", normals, bias_accelerations)
        + namespace.einsum("...pi,...i->...p", normal_jacobian, acceleration_drift)
        + (parameters.alpha1 + parameters.alpha2) * hdot
        + parameters.alpha1 * parameters.alpha2 * h
    )
    return BarrierConstraints(
        coefficient=namespace.einsum(
            "...pi,...ij->...pj", normal_jacobian, acceleration_control
        ),
        constant=constant,
        clearance=h,
        h=h,
        psi1=hdot + parameters.alpha1 * h,
    )


def geometry_constraints[Array](
    *,
    positions: Array,
    jacobians: Array,
    bias_accelerations: Array,
    velocity: Array,
    acceleration_drift: Array,
    acceleration_control: Array,
    layout: ConstraintLayout[Array],
    parameters: CbfParameters,
    namespace: Any = np,
) -> BarrierConstraints[Array]:
    """Apply the common pair layout to scalar or batched model evaluations."""
    common = dict(
        velocity=velocity,
        acceleration_drift=acceleration_drift,
        acceleration_control=acceleration_control,
        parameters=parameters,
        namespace=namespace,
    )
    rows = []
    if len(layout.first):
        first, second = layout.first, layout.second
        rows.append(
            sphere_constraints(
                displacement=positions[..., first, :] - positions[..., second, :],
                relative_jacobian=jacobians[..., first, :, :]
                - jacobians[..., second, :, :],
                relative_bias_acceleration=(
                    bias_accelerations[..., first, :]
                    - bias_accelerations[..., second, :]
                ),
                separation=layout.separation,
                **common,
            )
        )
    if len(layout.plane_spheres):
        indices = layout.plane_spheres
        rows.append(
            plane_constraints(
                positions=positions[..., indices, :],
                jacobians=jacobians[..., indices, :, :],
                bias_accelerations=bias_accelerations[..., indices, :],
                normals=layout.plane_normals,
                offsets_with_radius=layout.plane_offsets,
                **common,
            )
        )
    return BarrierConstraints(
        **{
            name: namespace.concatenate(
                tuple(getattr(row, name) for row in rows),
                axis=-2 if name == "coefficient" else -1,
            )
            for name in ("coefficient", "constant", "clearance", "h", "psi1")
        }
    )
