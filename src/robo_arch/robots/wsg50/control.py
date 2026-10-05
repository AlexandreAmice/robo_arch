"""Nominal aperture and compliant mechanical-centering control in SI units."""

import numpy as np


def effort(
    q: np.ndarray, v: np.ndarray, aperture: float, force_limit: float = 20.0
) -> np.ndarray:
    """Return two finger forces; aperture is signed joint travel, not tip clearance.

    Mechanical synchronization is approximated by finite centering feedback.
    The opening/closing force difference is limited; the source asset separately
    bounds each actuator. This is a simulation model, not a hardware driver.
    """
    centering = -2000.0 * (q[0] + q[1]) - 20.0 * (v[0] + v[1])
    opening = np.clip(
        300.0 * (aperture + q[0] - q[1]) + 10.0 * (v[0] - v[1]),
        -force_limit,
        force_limit,
    )
    return np.clip(
        np.array([centering - opening, centering + opening]) / 2, -80.0, 80.0
    )
