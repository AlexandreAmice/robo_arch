"""Interpret recorded row identities and safety margins independently of rendering."""

import numpy as np
import pytest

from robo_arch.scenarios.camera_protection.plotting import clearance_series


def description():
    return {
        # Deliberately interleaved: metadata identity, not a guessed row offset,
        # determines what is reported as floor avoidance.
        "pair_names": ("tool|ground", "camera|obstacle", "wrist|ground"),
        "pairs": [{"first": "camera", "second": "obstacle", "margin": 0.01}],
        "plane_pairs": [
            {"sphere": "wrist", "plane": "ground", "margin": 0.02},
            {"sphere": "tool", "plane": "ground", "margin": 0.01},
        ],
    }


def test_ground_series_excludes_other_pairs_and_adds_each_margin_before_minimum():
    data = np.zeros((2, 5 * 3 + 3))
    data[:, :3] = [[0.001, -0.1, 0.003], [0.010, 0.2, 0.002]]
    data[0, 4 * 3 + 1] = 1  # An active obstacle constraint is not ground activity.
    data[1, 4 * 3 + 2] = 1
    data[:, 5 * 3] = [2.5, 3.0]
    series = clearance_series(data, filtered=True, description=description())
    np.testing.assert_allclose(series["ground_clearance"], [0.001, 0.002])
    np.testing.assert_allclose(series["sphere_clearance"], [-0.1, 0.2])
    np.testing.assert_allclose(series["ground_gap"], [0.011, 0.020])
    np.testing.assert_array_equal(series["ground_active"], [False, True])
    np.testing.assert_allclose(series["torque_correction"], [2.5, 3.0])


def test_baseline_preserves_negative_ground_clearance():
    series = clearance_series(
        np.array([[-0.012, 0.1, 0.005]]), filtered=False, description=description()
    )
    np.testing.assert_allclose(series["ground_clearance"], [-0.012])
    np.testing.assert_allclose(series["ground_gap"], [-0.002])
    assert "ground_active" not in series
    assert "torque_correction" not in series


def test_missing_description_uses_unlabelled_legacy_minimum():
    series = clearance_series(np.array([[0.2, -0.1]]), filtered=False)
    np.testing.assert_allclose(series["all_clearance"], [-0.1])
    assert "ground_clearance" not in series


def test_metadata_row_count_mismatch_cannot_mislabel_floor_evidence():
    with pytest.raises(ValueError, match="every recorded constraint row"):
        clearance_series(np.zeros((1, 2)), filtered=False, description=description())


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
