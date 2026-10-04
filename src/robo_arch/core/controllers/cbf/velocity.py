"""Shared first-order joint-velocity barriers for bounded effort projection."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VelocityBounds[Array]:
    names: tuple[str, ...]
    indices: Array
    signs: Array
    offsets: Array

    def slack(self, velocity: Array) -> Array:
        """Signed distance inside each finite bound, in rad/s or m/s."""
        return self.signs * velocity[..., self.indices] + self.offsets

    def constraints(
        self, velocity: Array, drift: Array, control: Array, gain: float
    ) -> tuple[Array, Array]:
        """Enforce hdot + gain*h >= 0 with identical NumPy/Torch equations."""
        return (
            self.signs[:, None] * control[..., self.indices, :],
            self.signs * drift[..., self.indices] + gain * self.slack(velocity),
        )


def compile_velocity_bounds(
    joints: tuple[str, ...], lower, upper
) -> VelocityBounds[np.ndarray]:
    """Retain finite model bounds in joint order; unbounded sides add no row."""
    lower, upper = np.asarray(lower), np.asarray(upper)
    if (
        lower.shape != (len(joints),)
        or upper.shape != lower.shape
        or np.isnan(lower).any()
        or np.isnan(upper).any()
        or np.isposinf(lower).any()
        or np.isneginf(upper).any()
        or np.any(lower >= upper)
    ):
        raise ValueError("Joint velocity bounds must be ordered and non-NaN")
    names, indices, signs, offsets = [], [], [], []
    for i, name in enumerate(joints):
        for side, bound, sign in (("lower", lower[i], 1), ("upper", upper[i], -1)):
            if np.isfinite(bound):
                names.append(f"{name}/velocity/{side}")
                indices.append(i)
                signs.append(sign)
                offsets.append(-sign * bound)
    return VelocityBounds(
        tuple(names),
        np.array(indices, dtype=int),
        np.array(signs, dtype=float),
        np.array(offsets, dtype=float),
    )
