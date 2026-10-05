"""Import the installed or Bazel-provided extension without implicit compilation."""

from importlib.util import find_spec
from inspect import cleandoc

import numpy as np

from robo_arch.core.controllers.joint_pd._docstrings import JOINT_PD


def compute(
    q: np.ndarray,
    v: np.ndarray,
    q_des: np.ndarray,
    v_des: np.ndarray,
    feedforward: np.ndarray,
    kp: np.ndarray,
    kd: np.ndarray,
) -> np.ndarray:
    """Python array interface: inputs are one-dimensional, C-contiguous CPU float64
    arrays. No implicit dtype or layout conversion is performed. The returned
    writable NumPy array owns independent storage; inputs are not modified.
    The GIL is held during computation. Invalid inputs raise ValueError (values)
    or TypeError (array representation); nonfinite output raises OverflowError.
    """
    if find_spec("robo_arch_native") is None:
        raise ModuleNotFoundError(
            "Native controller is not installed. Direct scenario scripts refresh it "
            "automatically. For IDEs/notebooks, run: "
            "uv run tools/native/install.py --profile drake "
            "(or --profile isaac), then restart the Python process."
        )
    from robo_arch_native._joint_pd import compute as native_compute

    return native_compute(q, v, q_des, v_des, feedforward, kp, kd)


# Compose documentation without importing the optional native extension.
compute.__doc__ = JOINT_PD + "\n\n" + cleandoc(compute.__doc__)
