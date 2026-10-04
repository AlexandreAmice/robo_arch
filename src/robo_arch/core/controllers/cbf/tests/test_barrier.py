"""Independent differentiation checks for the sphere separation constraint."""

import numpy as np
import pytest

from robo_arch.core.controllers.cbf.barrier import plane_constraints, sphere_constraint
from robo_arch.core.controllers.cbf.definition import (
    CbfParameters,
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)


def test_relative_moving_centers_match_finite_differences():
    q = np.array([0.4, -0.3])
    v = np.array([0.7, -0.2])
    drift = np.array([0.2, 0.3])
    control = np.array([[1.0, 0.1], [0.1, 0.8]])
    effort = np.array([-0.4, 0.8])
    acceleration = drift + control @ effort
    parameters = CbfParameters(alpha1=3, alpha2=4)

    def displacement(position):
        return np.array([np.sin(position[0]) - position[1], np.cos(position[0]), 0])

    jacobian = np.array([[np.cos(q[0]), -1], [-np.sin(q[0]), 0], [0, 0]])
    bias = np.array([-np.sin(q[0]), -np.cos(q[0]), 0]) * v[0] ** 2
    row = sphere_constraint(
        displacement=displacement(q),
        relative_jacobian=jacobian,
        relative_bias_acceleration=bias,
        velocity=v,
        acceleration_drift=drift,
        acceleration_control=control,
        separation=0.25,
        parameters=parameters,
    )

    def h(time):
        d = displacement(q + v * time + 0.5 * acceleration * time**2)
        return d @ d - 0.25**2

    dt = 1e-4
    first = (h(dt) - h(-dt)) / (2 * dt)
    second = (h(dt) - 2 * h(0) + h(-dt)) / dt**2
    expected = second + 7 * first + 12 * h(0)
    assert row.coefficient @ effort + row.constant == pytest.approx(expected, abs=1e-7)
    assert row.psi1 == pytest.approx(first + 3 * h(0), abs=1e-8)


@pytest.mark.parametrize("radius", [0, -1, float("nan"), float("inf")])
def test_invalid_sphere_radius(radius):
    with pytest.raises(ValueError, match="radius"):
        Sphere("camera", "world", (0, 0, 0), radius)


def test_invalid_pair_and_parameters():
    with pytest.raises(ValueError, match="distinct"):
        SpherePair("camera", "camera")
    with pytest.raises(ValueError, match="nonnegative"):
        SpherePair("camera", "wall", -0.1)
    with pytest.raises(ValueError, match="positive"):
        CbfParameters(alpha1=0)


@pytest.mark.parametrize(
    "normal", [(0, 0, 0), (0, 0, 2), (0, np.nan, 1), (0, np.inf, 1), (0, 1)]
)
def test_plane_requires_finite_unit_normal(normal):
    with pytest.raises(ValueError, match="normal"):
        Plane("floor", normal, 0)


def test_plane_offset_and_pair_validation():
    with pytest.raises(ValueError, match="offset"):
        Plane("floor", (0, 0, 1), np.inf)
    with pytest.raises(ValueError, match="distinct"):
        SpherePlanePair("floor", "floor")
    with pytest.raises(ValueError, match="nonnegative"):
        SpherePlanePair("camera", "floor", -0.1)


def test_tilted_plane_derivatives_match_finite_difference():
    q, velocity, drift, control, effort = 0.3, 0.7, -0.4, 1.3, -0.5
    normal = np.array([1, 1, 0]) / np.sqrt(2)
    acceleration = drift + control * effort
    parameters = CbfParameters(alpha1=3, alpha2=4)

    def position(angle):
        return np.array([np.sin(angle), np.cos(angle), 0])

    (row,) = plane_constraints(
        positions=position(q)[None, :],
        jacobians=np.array([[[np.cos(q)], [-np.sin(q)], [0]]]),
        bias_accelerations=(-position(q) * velocity**2)[None, :],
        normals=normal[None, :],
        offsets_with_radius=np.array([0.2]),
        velocity=np.array([velocity]),
        acceleration_drift=np.array([drift]),
        acceleration_control=np.array([[control]]),
        parameters=parameters,
    ).rows()

    def h(time):
        return (
            normal @ position(q + time * velocity + 0.5 * time**2 * acceleration) - 0.2
        )

    dt = 1e-4
    first = (h(dt) - h(-dt)) / (2 * dt)
    second = (h(dt) - 2 * h(0) + h(-dt)) / dt**2
    assert row.h == pytest.approx(h(0))
    assert row.clearance == row.h
    assert row.coefficient @ [effort] + row.constant == pytest.approx(
        second + 7 * first + 12 * h(0), abs=1e-7
    )


def test_velocity_bounds_keep_one_sided_limits_and_unbounded_joints():
    from robo_arch.core.controllers.cbf.velocity import compile_velocity_bounds

    bounds = compile_velocity_bounds(
        ("lower", "upper", "free"), [-2, -np.inf, -np.inf], [np.inf, 3, np.inf]
    )
    assert bounds.names == ("lower/velocity/lower", "upper/velocity/upper")
    velocity = np.array([[-1.9, 2.8, 100], [-1.0, 2.0, -100]])
    drift = np.array([[1, 2, 3], [4, 5, 6]])
    control = np.broadcast_to(np.diag([2, 3, 4]), (2, 3, 3))
    a, b = bounds.constraints(velocity, drift, control, gain=20)
    np.testing.assert_allclose(bounds.slack(velocity), [[0.1, 0.2], [1, 1]])
    np.testing.assert_allclose(a, [[[2, 0, 0], [0, -3, 0]]] * 2)
    np.testing.assert_allclose(b, [[3, 2], [24, 15]])


@pytest.mark.parametrize(
    "lower,upper", [(1, 0), (np.nan, 1), (np.inf, np.inf), (-np.inf, -np.inf)]
)
def test_invalid_velocity_bounds_raise(lower, upper):
    from robo_arch.core.controllers.cbf.velocity import compile_velocity_bounds

    with pytest.raises(ValueError, match="velocity bounds"):
        compile_velocity_bounds(("joint",), [lower], [upper])


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
