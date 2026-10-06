"""PD feedback on CPU or GPU tensors, with caller-provided feedforward.

Units and joint order match the native implementation. Inputs are borrowed;
outputs own their storage. Leading dimensions are batches, the last is joints.
Torch is supplied by the selected runtime profile, not declaration imports.
"""

import torch
from torch import Tensor

from robo_arch.core.controllers.feedback import feedback


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
    effort = feedback(q, v, q_des, v_des, kp, kd) + feedforward
    return effort.clamp(-limits, limits)


class TensorJointPd:
    """Gravity-fed PD using the same independent nominal model as CPU control."""

    def __init__(self, model, parameters):
        if len(parameters.kp) != model.count:
            raise ValueError("PD gain count must match the robot model")
        self.model = model
        self.kp = torch.tensor(parameters.kp, dtype=model.dtype, device=model.device)
        self.kd = torch.tensor(parameters.kd, dtype=model.dtype, device=model.device)
        self.limits = model.limits

    def effort(self, state, desired_position, desired_velocity, *, evaluation=None):
        count = self.model.count
        q, v = state[:, :count], state[:, count:]
        gravity = self.model.evaluate(
            torch.cat((q, torch.zeros_like(v)), -1)
        ).bias_force
        return compute(
            q,
            v,
            desired_position,
            desired_velocity,
            gravity,
            self.kp,
            self.kd,
            self.limits,
        )
