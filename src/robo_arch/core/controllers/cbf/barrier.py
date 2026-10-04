"""Second-order sphere separation constraints, independent of a physics SDK."""

from dataclasses import dataclass

import numpy as np

from robo_arch.core.controllers.cbf.definition import CbfParameters


@dataclass(frozen=True)
class BarrierConstraint:
    """One affine inequality ``coefficient @ effort + constant >= 0``.

    :param coefficient: Control coefficients, shape (actuators,).
    :param constant: Contribution independent of effort.
    :param clearance: Signed surface separation after margin, in metres.
    :param h: Barrier value; m² for sphere pairs and m for planes.
    :param psi1: ``hdot + alpha1*h``; m²/s for sphere pairs and m/s for planes.

    The inequality residual has units m²/s² (sphere pairs) or m/s² (planes).
    Direct construction stores the supplied array without copying. The numerical
    scalar builder and :meth:`BarrierConstraints.rows` return owned coefficients.
    Freezing the record does not make its arrays read-only.
    """

    coefficient: np.ndarray
    constant: float
    clearance: float
    h: float
    psi1: float


@dataclass(frozen=True)
class BarrierConstraints:
    """Vectorized barrier rows, with protection pair as the leading axis.

    :param coefficient: Shape (pairs, actuators), for ``coefficient @ effort``.
    :param constant: Shape (pairs,), added to the effort contribution.
    :param clearance: Shape (pairs,), signed clearance after margin, in metres.
    :param h: Shape (pairs,), in m² for sphere pairs or m for planes.
    :param psi1: Shape (pairs,), in m²/s for sphere pairs or m/s for planes.

    Direct construction stores supplied arrays without copying. For builders,
    sphere_constraints creates fresh
    outputs; plane_constraints shares its ``clearance`` and ``h`` array. Arrays
    remain mutable. Use :meth:`rows` for independent scalar records.
    """

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
    """Assemble sphere-pair barriers using common nominal dynamics.

    Let P be the number of pairs, V generalized velocities and U actuator efforts.
    All point kinematics are expressed in world, with a consistent first-minus-
    second convention. Both centers may move. Dynamics use
    ``vdot = acceleration_drift + acceleration_control @ effort``.

    :param displacement: Relative center positions, shape (P, 3), in metres.
    :param relative_jacobian: Maps generalized velocity to relative center
        velocity; shape (P, 3, V).
    :param relative_bias_acceleration: Relative ``Jdot @ v``, shape (P, 3), m/s².
    :param velocity: Generalized velocities, shape (V,), in joint-specific units.
    :param acceleration_drift: Nominal acceleration at zero effort, shape (V,).
    :param acceleration_control: Acceleration per actuator effort, shape (V, U).
    :param separation: Sum of both radii and extra margin, shape (P,), in metres.
    :param parameters: Positive barrier gains; residual tolerance is not used here.
    :returns: Fresh rows enforcing ``hddot + (alpha1 + alpha2)*hdot
        + alpha1*alpha2*h >= 0``, with ``h = dot(d, d) - separation**2``.

    Inputs are borrowed and not modified. Supply finite NumPy arrays of the
    stated shapes; NumPy errors propagate, but this function does not validate
    finiteness, geometry, initial feasibility or effort limits. It computes rows,
    not a safe command, and the leading axis represents pairs, not environments.
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
    """Scalar-pair interface to :func:`sphere_constraints`.

    Use the same world-frame convention, dynamics and units as the vectorized
    function, removing its pair axis: ``displacement`` and
    ``relative_bias_acceleration`` have shape (3,), ``relative_jacobian`` has
    shape (3, V), and ``separation`` is a scalar in metres. ``velocity`` and
    ``acceleration_drift`` retain shape (V,), ``acceleration_control`` is (V, U),
    and ``parameters`` supplies the same gains.

    Returns an owned BarrierConstraint. Inputs are not modified; shape errors
    propagate. This adds no finiteness or feasibility validation.
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
    """Assemble sphere-to-fixed-plane rows using linear signed clearance.

    P denotes pairs, V generalized velocities and U actuator efforts. All point
    kinematics and plane normals are world-expressed. The allowed halfspace is
    ``normal @ center >= offset + radius + margin``.

    :param positions: Sphere centers, shape (P, 3), in metres.
    :param jacobians: Center-velocity Jacobians, shape (P, 3, V).
    :param bias_accelerations: Center ``Jdot @ v``, shape (P, 3), in m/s².
    :param normals: Unit normals into permitted halfspaces, shape (P, 3).
    :param offsets_with_radius: Plane offset plus radius and margin, shape (P,), m.
    :param velocity: Generalized velocities, shape (V,).
    :param acceleration_drift: Nominal zero-effort acceleration, shape (V,).
    :param acceleration_control: Acceleration per actuator effort, shape (V, U).
    :param parameters: Positive barrier gains; residual tolerance is not used here.
    :returns: Rows for the same second-order inequality as sphere_constraints;
        ``h`` is signed clearance in metres and aliases the ``clearance`` output.

    Inputs are not modified. Callers provide finite, correctly shaped arrays and
    normalized normals; no explicit input or feasibility validation occurs here.
    NumPy errors propagate. Outputs otherwise have independent storage.
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
