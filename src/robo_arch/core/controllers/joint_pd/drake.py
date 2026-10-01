"""Native PD feedback and independent Drake-model gravity feedforward."""

from collections.abc import Callable

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, LeafSystem

from robo_arch.core.controllers.joint_pd.definition import JointPdParameters
from robo_arch.core.controllers.joint_pd.native import compute


def make_policy(
    *,
    model: MultibodyPlant,
    parameters: JointPdParameters,
    joints: tuple[str, ...],
    desired_state: Callable[[float], np.ndarray],
) -> Callable[[np.ndarray, float], np.ndarray]:
    """Own a model context; return effort in joint order, clipped to model limits."""
    names = tuple(
        model.get_joint_actuator(i).joint().name()
        for i in model.GetJointActuatorIndices()
    )
    count = len(joints)
    if names != joints or model.num_positions() != count:
        raise ValueError("PD controller model must match the declared joint order")
    if len(parameters.kp) != count:
        raise ValueError("PD gain count must match the robot model")
    context = model.CreateDefaultContext()
    kp = np.asarray(parameters.kp, dtype=float)
    kd = np.asarray(parameters.kd, dtype=float)
    limits = np.array(
        [
            model.get_joint_actuator(i).effort_limit()
            for i in model.GetJointActuatorIndices()
        ]
    )

    def command(state: np.ndarray, time: float) -> np.ndarray:
        state = np.ascontiguousarray(state, dtype=float)
        reference = np.ascontiguousarray(desired_state(time), dtype=float)
        if state.shape != (2 * count,) or reference.shape != (2 * count,):
            raise ValueError("PD state/reference must contain ordered [q, v]")
        model.SetPositionsAndVelocities(context, state)
        feedforward = -model.CalcGravityGeneralizedForces(context)
        effort = compute(
            state[:count],
            state[count:],
            reference[:count],
            reference[count:],
            feedforward,
            kp,
            kd,
        )
        return np.clip(effort, -limits, limits)

    # Fail at construction if the extension is missing or incompatible.
    command(np.zeros(2 * count), 0.0)
    return command


class JointPdSystem(LeafSystem):
    """Drake port adapter; the C++ function is the only feedback implementation."""

    def __init__(
        self,
        *,
        model: MultibodyPlant,
        parameters: JointPdParameters,
        joints: tuple[str, ...],
    ):
        super().__init__()
        count = len(joints)
        self._state = self.DeclareVectorInputPort("estimated_state", 2 * count)
        self._reference = self.DeclareVectorInputPort("desired_state", 2 * count)
        # Each output evaluation supplies its reference explicitly. This nominal
        # context belongs to this system, never to the physical simulation.
        self._desired = np.zeros(2 * count)
        self._command = make_policy(
            model=model,
            parameters=parameters,
            joints=joints,
            desired_state=lambda time: self._desired,
        )
        self.DeclareVectorOutputPort("effort", BasicVector(count), self._output)

    def _output(self, context, output):
        self._desired = self._reference.Eval(context)
        output.SetFromVector(
            self._command(self._state.Eval(context), context.get_time())
        )
