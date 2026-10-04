"""Shared projection checks include every row, with scalar and batch ownership."""

import numpy as np
import pytest

from robo_arch.core.controllers.cbf.barrier import BarrierConstraints
from robo_arch.core.controllers.cbf.errors import CbfFailure
from robo_arch.core.controllers.cbf.filter import project, validate_initial_state


@pytest.mark.parametrize("batch_shape", [(), (2,)])
def test_velocity_row_is_checked_even_when_geometry_is_safe(batch_shape):
    coefficient = np.ones((*batch_shape, 1, 1))
    constant = np.ones((*batch_shape, 1))
    rows = BarrierConstraints(coefficient, constant, constant, constant, constant)
    # Geometry requires u >= -1, velocity requires u <= 0.25.
    with pytest.raises(CbfFailure, match="residual check"):
        project(
            rows=rows,
            velocity_coefficient=-coefficient,
            velocity_constant=constant * 0.25,
            velocity_slack=constant,
            nominal=constant,
            limits=np.array([2.0]),
            solve=lambda a, b, u: u,
            tolerance=1e-6,
        )
    command = constant * 0.25
    result = project(
        rows=rows,
        velocity_coefficient=-coefficient,
        velocity_constant=constant * 0.25,
        velocity_slack=constant,
        nominal=constant,
        limits=np.array([2.0]),
        solve=lambda a, b, u: command,
        tolerance=1e-6,
    )
    assert result.diagnostics.shape == (*batch_shape, 8)
    np.testing.assert_array_equal(result.velocity_residual, np.zeros_like(constant))
    np.testing.assert_array_equal(result.diagnostics[..., 3], 1.25)
    np.testing.assert_array_equal(result.diagnostics[..., 5], 0.75)
    command[:] = 99
    constant[:] = 99
    np.testing.assert_array_equal(result.effort, 0.25)
    np.testing.assert_array_equal(result.velocity_slack, 1.0)


@pytest.mark.parametrize("bad_value", [np.nan, np.inf, -0.01])
def test_initial_domain_rejects_nonfinite_or_negative_barriers(bad_value):
    rows = BarrierConstraints(
        np.ones((1, 1)), np.ones(1), np.ones(1), np.array([bad_value]), np.ones(1)
    )
    with pytest.raises(CbfFailure, match="initial state"):
        validate_initial_state(
            rows, np.zeros(0), pair_names=("camera|floor",), velocity_names=()
        )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
