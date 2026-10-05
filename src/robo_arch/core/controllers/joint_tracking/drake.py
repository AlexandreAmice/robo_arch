"""Drake inverse dynamics with model and tuning supplied by the owning setup."""

from collections.abc import Callable

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import BasicVector, DiagramBuilder, InputPort, LeafSystem

from robo_arch.core.controllers.dynamics.drake import build_tensor_model
from robo_arch.core.controllers.feedback import inverse_dynamics
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters
from robo_arch.core.worlds.drake.scene import DrakeScene


def build(
    *,
    model: MultibodyPlant,
    parameters: JointTrackingParameters,
    joints: tuple[str, ...],
) -> "JointTrackingSystem":
    """Return a controller; the caller adds it to a DiagramBuilder.

    Input 0 is [q, v], input 1 is [q_desired, v_desired], output 0 is effort.
    No integral gain or feedforward-acceleration input is enabled. The model is
    a separate fixed-base, fully actuated arm model, not the simulation plant.
    """
    if not isinstance(model, MultibodyPlant) or not model.is_finalized():
        raise ValueError("joint_tracking requires a finalized Drake controller model")
    count = model.num_positions()
    if model.num_velocities() != count or model.num_actuated_dofs() != count:
        raise ValueError("joint_tracking requires a fixed-base, fully actuated model")
    if len(parameters.kp) != count:
        raise ValueError("Controller gain count must match the robot model")
    actuator_joints = tuple(
        model.get_joint_actuator(index).joint().name()
        for index in model.GetJointActuatorIndices()
    )
    if actuator_joints != joints:
        raise ValueError("Controller model joint order must match the expected joints")
    return JointTrackingSystem(model, parameters, joints)


class JointTrackingSystem(LeafSystem):
    """Native ports around the shared numerical law and independent JAX model."""

    def __init__(self, model, parameters, joints):
        super().__init__()
        self.model = build_tensor_model(
            model, joints, ("world",), ((0, 0, 0),), device="cpu"
        )
        self.kp, self.kd = np.asarray(parameters.kp), np.asarray(parameters.kd)
        self._state = self.DeclareVectorInputPort("estimated_state", 2 * len(joints))
        self._reference = self.DeclareVectorInputPort("desired_state", 2 * len(joints))
        self._effort = self.DeclareVectorOutputPort(
            "effort", BasicVector(len(joints)), self._output
        )

    def get_input_port_estimated_state(self):
        return self._state

    def get_input_port_desired_state(self):
        return self._reference

    def get_output_port_control(self):
        return self._effort

    def _output(self, context, output):
        state = self._state.Eval(context)
        reference = self._reference.Eval(context)
        count = self.model.count
        data = self.model.evaluate_numpy(state[None, :])
        output.SetFromVector(
            inverse_dynamics(
                state,
                reference[:count],
                reference[count:],
                self.kp,
                self.kd,
                data.mass[0],
                data.bias_force[0],
            )
        )


def connect(
    builder: DiagramBuilder,
    scene: DrakeScene,
    *,
    robot: str,
    parameters: JointTrackingParameters,
    joints: tuple[str, ...],
) -> dict[str, InputPort]:
    """Wire this controller to its robot; leave task references exposed."""
    controller = builder.AddSystem(
        build(
            model=scene.controller_models[robot],
            parameters=parameters,
            joints=joints,
        )
    )
    controller.set_name(f"{robot}/joint_tracking")
    instance = scene.robots[robot]
    builder.Connect(
        scene.plant.get_state_output_port(instance),
        controller.get_input_port_estimated_state(),
    )
    builder.Connect(
        controller.get_output_port_control(),
        scene.plant.get_actuation_input_port(instance),
    )
    return {"desired_state": controller.get_input_port_desired_state()}


def make_policy(
    *,
    model: MultibodyPlant,
    parameters: JointTrackingParameters,
    joints: tuple[str, ...],
    desired_state: Callable[[float], np.ndarray],
) -> Callable[[np.ndarray, float], np.ndarray]:
    """Evaluate the same controller on CPU for externally supplied [q, v] state.

    Each factory call owns a separate controller and context. No Drake physics
    simulator or copy of the control equations is involved.
    """
    controller = build(model=model, parameters=parameters, joints=joints)
    context = controller.CreateDefaultContext()

    def command(state: np.ndarray, time: float) -> np.ndarray:
        # Keep the borrowed dynamics model alive alongside the native controller.
        if len(state) != model.num_positions() + model.num_velocities():
            raise ValueError("Observation must contain the robot's ordered [q, v]")
        context.SetTime(time)
        controller.get_input_port_estimated_state().FixValue(context, state)
        controller.get_input_port_desired_state().FixValue(context, desired_state(time))
        return controller.get_output_port_control().Eval(context).copy()

    return command
