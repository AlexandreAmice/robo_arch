"""Second-order sphere separation constraints, independent of a physics SDK."""

from dataclasses import dataclass

import numpy as np

from robo_arch.core.controllers.cbf.definition import CbfParameters


@dataclass(frozen=True)
class BarrierConstraint:
    """Affine inequality ``coefficient @ effort + constant >= 0``."""

    coefficient: np.ndarray
    constant: float
    clearance: float
    h: float
    psi1: float


@dataclass(frozen=True)
class BarrierConstraints:
    """One row per protection pair, with all numerical fields stored as arrays."""

    coefficient: np.ndarray
    constant: np.ndarray
    clearance: np.ndarray
    h: np.ndarray
    psi1: np.ndarray

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


def sphere_constraints(
    *,
    displacement: np.ndarray,
    relative_jacobian: np.ndarray,
    relative_bias_acceleration: np.ndarray,
    velocity: np.ndarray,
    acceleration_drift: np.ndarray,
    acceleration_control: np.ndarray,
    separation: np.ndarray,
    parameters: CbfParameters,
) -> BarrierConstraints:
    """Assemble independent pair rows together, with pair as the leading axis.

    Positions/biases have shape (pairs, 3), Jacobians (pairs, 3, velocities),
    and separation (pairs,). Dynamics are shared by every pair.
    """
    d, j = displacement, relative_jacobian
    center_velocity = j @ velocity
    distance_squared = np.einsum("pi,pi->p", d, d)
    h = distance_squared - separation**2
    two_d_j = 2 * np.einsum("pi,pij->pj", d, j)
    hdot = two_d_j @ velocity
    acceleration = relative_bias_acceleration + j @ acceleration_drift
    constant = (
        2 * np.einsum("pi,pi->p", center_velocity, center_velocity)
        + 2 * np.einsum("pi,pi->p", d, acceleration)
        + (parameters.alpha1 + parameters.alpha2) * hdot
        + parameters.alpha1 * parameters.alpha2 * h
    )
    return BarrierConstraints(
        coefficient=two_d_j @ acceleration_control,
        constant=constant,
        clearance=np.sqrt(distance_squared) - separation,
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


def plane_constraints(
    *,
    positions: np.ndarray,
    jacobians: np.ndarray,
    bias_accelerations: np.ndarray,
    normals: np.ndarray,
    offsets_with_radius: np.ndarray,
    velocity: np.ndarray,
    acceleration_drift: np.ndarray,
    acceleration_control: np.ndarray,
    parameters: CbfParameters,
) -> BarrierConstraints:
    """Assemble sphere-to-fixed-plane rows using linear signed distance.

    All positions, normals and biases are world expressed, with pair first.
    Offsets include plane offset, sphere radius and extra margin. Unit normals
    make h a distance in meters, unlike squared-distance sphere-pair barriers.
    """
    h = np.einsum("pi,pi->p", normals, positions) - offsets_with_radius
    normal_jacobian = np.einsum("pi,pij->pj", normals, jacobians)
    hdot = normal_jacobian @ velocity
    constant = (
        np.einsum("pi,pi->p", normals, bias_accelerations)
        + normal_jacobian @ acceleration_drift
        + (parameters.alpha1 + parameters.alpha2) * hdot
        + parameters.alpha1 * parameters.alpha2 * h
    )
    return BarrierConstraints(
        coefficient=normal_jacobian @ acceleration_control,
        constant=constant,
        clearance=h,
        h=h,
        psi1=hdot + parameters.alpha1 * h,
    )
