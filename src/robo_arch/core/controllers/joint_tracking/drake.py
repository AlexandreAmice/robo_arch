"""Drake inverse dynamics with model and tuning supplied by the owning setup."""

from collections.abc import Callable

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.controllers import InverseDynamicsController
from pydrake.systems.framework import DiagramBuilder, InputPort

from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters
from robo_arch.core.worlds.drake.scene import DrakeScene


def build(
    *,
    model: MultibodyPlant,
    parameters: JointTrackingParameters,
    joints: tuple[str, ...],
) -> InverseDynamicsController:
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
    return InverseDynamicsController(
        model,
        kp=parameters.kp,
        ki=[0.0] * count,
        kd=parameters.kd,
        has_reference_acceleration=False,
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
