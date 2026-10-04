"""Exercise compiled feedback, memory ownership and strict array boundaries."""

import gc

import numpy as np
import pytest

from robo_arch.core.controllers.joint_pd.native import compute


def test_shared_documentation_and_python_addendum():
    from robo_arch_native._joint_pd import compute as bound_compute

    from robo_arch.core.controllers.joint_pd._docstrings import JOINT_PD

    assert JOINT_PD in bound_compute.__doc__
    assert compute.__doc__.count(JOINT_PD) == 1
    assert "Python array interface" in compute.__doc__
    assert ":param q: Measured joint positions in rad." in compute.__doc__


@pytest.mark.parametrize("count", [6, 7])
def test_feedback_and_owned_result(count):
    q = np.linspace(-0.2, 0.3, count)
    v = np.linspace(0.1, -0.1, count)
    target = np.full(count, 0.25)
    vd = np.full(count, 0.02)
    ff = np.linspace(1, 3, count)
    kp = np.full(count, 20.0)
    kd = np.full(count, 4.0)
    expected = ff + kp * (target - q) + kd * (vd - v)
    q.flags.writeable = False
    result = compute(q, v, target, vd, ff, kp, kd)
    del q, v, target, vd, ff, kp, kd
    gc.collect()
    np.testing.assert_allclose(result, expected, atol=1e-14)
    result[:] = 0  # Output storage is writable and does not borrow any input.


@pytest.mark.parametrize("bad", [np.ones(5), np.full(6, np.nan)])
def test_bad_sizes_and_values(bad):
    values = [np.ones(6) for _ in range(7)]
    values[0] = bad
    with pytest.raises(ValueError):
        compute(*values)


@pytest.mark.parametrize(
    "bad", [np.ones(6, dtype=np.float32), np.ones(12)[::2], np.ones((2, 3))]
)
def test_no_implicit_array_conversion(bad):
    values = [np.ones(6) for _ in range(7)]
    values[0] = bad
    with pytest.raises(TypeError):
        compute(*values)


def test_negative_gain_rejected():
    values = [np.ones(6) for _ in range(7)]
    values[5][2] = -1
    with pytest.raises(ValueError, match="nonnegative"):
        compute(*values)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
