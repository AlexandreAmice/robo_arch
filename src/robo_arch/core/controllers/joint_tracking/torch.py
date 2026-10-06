"""Batched inverse dynamics using a caller-owned independent tensor model."""

import torch

from robo_arch.core.controllers.dynamics.torch import ModelEvaluation, TensorModel
from robo_arch.core.controllers.feedback import inverse_dynamics
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters


class TensorJointTracking:
    """Apply acceleration feedback and inverse dynamics in declared joint order.

    Reuse evaluation from the same state when composing with a model-based filter.
    The tensor model validates fixed-base full actuation and joint ordering.
    """

    def __init__(self, model: TensorModel, parameters: JointTrackingParameters):
        if len(parameters.kp) != model.count:
            raise ValueError("Controller gain count must match the robot model")
        self.model = model
        self.kp = torch.tensor(parameters.kp, device=model.device, dtype=model.dtype)
        self.kd = torch.tensor(parameters.kd, device=model.device, dtype=model.dtype)

    def effort(
        self,
        state: torch.Tensor,
        desired_position: torch.Tensor,
        desired_velocity: torch.Tensor,
        *,
        evaluation: ModelEvaluation | None = None,
    ) -> torch.Tensor:
        data = self.model.evaluate(state) if evaluation is None else evaluation
        return inverse_dynamics(
            state,
            desired_position,
            desired_velocity,
            self.kp,
            self.kd,
            data.mass,
            data.bias_force,
        )
