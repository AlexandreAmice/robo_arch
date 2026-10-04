"""PD feedback on CPU or GPU tensors, with caller-provided feedforward.

Units and joint order match the native implementation. Inputs are borrowed;
outputs own their storage. Leading dimensions are batches, the last is joints.
Torch is supplied by the selected runtime profile, not declaration imports.
"""

from torch import Tensor


def compute(
    q: Tensor,
    v: Tensor,
    q_des: Tensor,
    v_des: Tensor,
    feedforward: Tensor,
    kp: Tensor,
    kd: Tensor,
    limits: Tensor,
) -> Tensor:
    """Return effort clipped to symmetric limits, without synchronizing the device.

    Gains and limits broadcast across batch dimensions. Configuration owners
    validate finite nonnegative gains/limits once before entering the control loop.
    """
    if q.shape != v.shape or q.shape != q_des.shape or q.shape != v_des.shape:
        raise ValueError("PD state and reference shapes must match")
    if feedforward.shape != q.shape:
        raise ValueError("Feedforward shape must match joint state")
    effort = kp * (q_des - q) + kd * (v_des - v) + feedforward
    return effort.clamp(-limits, limits)
