"""The same quintic approach/retreat reference for scalar and batched worlds."""

from typing import Any

import numpy as np


def desired_state(
    time: Any,
    initial: Any,
    unsafe: Any,
    retreat: Any,
    *,
    retreat_time: float,
    transition_seconds: float,
    namespace: Any = np,
) -> tuple[Any, Any]:
    """Return position and velocity; time has arbitrary leading batch axes.

    Arrays belong to the caller's NumPy or Torch namespace and device. Joint
    vectors broadcast over time. Output joint axis is last, in rad and rad/s.
    """
    approaching = time < retreat_time
    elapsed = namespace.where(approaching, time, time - retreat_time)
    start = namespace.where(approaching[..., None], initial, unsafe)
    end = namespace.where(approaching[..., None], unsafe, retreat)
    u = namespace.clip(elapsed / transition_seconds, 0, 1)[..., None]
    blend = 10 * u**3 - 15 * u**4 + 6 * u**5
    speed = (30 * u**2 - 60 * u**3 + 30 * u**4) / transition_seconds
    return start + blend * (end - start), speed * (end - start)
