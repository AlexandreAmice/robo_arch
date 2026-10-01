"""Wire arm-tracking references and autonomy into an existing Drake scene."""

import numpy as np
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.controllers.joint_tracking.drake import connect
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.drake.scene import DrakeScene


def configure(
    builder: DiagramBuilder,
    scene: DrakeScene,
    run: RunConfiguration,
    parameters: dict,
    *,
    desired_positions: dict[str, tuple[float, ...]],
) -> None:
    definitions = scene.definitions
    for robot in resolve_devices(run.scene).robots:
        joints = definitions.robots[robot.model].joints
        model = scene.controller_models[robot.name]
        target = np.asarray(desired_positions[robot.name])
        if target.shape != (len(joints),) or not np.isfinite(target).all():
            raise ValueError(f"Desired positions must match {robot.name}'s joints")
        if not (
            np.all(target >= model.GetPositionLowerLimits())
            and np.all(target <= model.GetPositionUpperLimits())
        ):
            raise ValueError(f"Desired positions exceed {robot.name}'s joint limits")
        goal = builder.AddSystem(
            ConstantVectorSource(np.r_[target, np.zeros(len(joints))])
        )
        if run.autonomy.controller == "joint_pd":
            from robo_arch.core.controllers.joint_pd.drake import JointPdSystem

            controller = builder.AddSystem(
                JointPdSystem(
                    model=model, parameters=parameters[robot.name], joints=joints
                )
            )
            controller.set_name(robot.name + "/joint_pd")
            builder.Connect(
                scene.plant.get_state_output_port(scene.robots[robot.name]),
                controller.GetInputPort("estimated_state"),
            )
            builder.Connect(
                controller.get_output_port(),
                scene.plant.get_actuation_input_port(scene.robots[robot.name]),
            )
            reference = controller.GetInputPort("desired_state")
        else:
            ports = connect(
                builder,
                scene,
                robot=robot.name,
                parameters=parameters[robot.name],
                joints=joints,
            )
            reference = ports["desired_state"]
        builder.Connect(goal.get_output_port(), reference)
