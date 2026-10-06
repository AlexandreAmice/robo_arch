"""Map per-device references and ports into one connected nominal mechanism."""

from collections.abc import Mapping

import numpy as np

from robo_arch.core.controllers.selection import scalar_policy


def make_policy(algorithm, mechanism, *, parameters: Mapping, targets: Mapping):
    """Keep every actuated coordinate/coupling while ordering device-owned gains."""
    model = mechanism.plant
    count = model.num_positions()
    kp, kd, goal = (np.empty(count) for _ in range(3))
    if set(parameters) != set(mechanism.indices) or set(targets) != set(
        mechanism.indices
    ):
        raise ValueError("Parameters and references must name every mechanism device")
    for name, indices in mechanism.indices.items():
        q = np.asarray(indices.q)
        parameter = parameters[name]
        target = np.asarray(targets[name], dtype=float)
        if target.shape != q.shape or len(parameter.kp) != len(q):
            raise ValueError(f"Gains and target must match {name}'s joints")
        goal[q], kp[q], kd[q] = target, parameter.kp, parameter.kd
    if (
        not np.isfinite(goal).all()
        or np.any(goal < model.GetPositionLowerLimits())
        or np.any(goal > model.GetPositionUpperLimits())
    ):
        raise ValueError("Task target must match the mechanism's limits")
    schema = type(next(iter(parameters.values())))
    joint_names = tuple(
        model.get_joint_actuator(i).joint().name()
        for i in model.GetJointActuatorIndices()
    )
    reference = np.r_[goal, np.zeros(count)]
    return scalar_policy(
        algorithm,
        model=model,
        parameters=schema(kp=tuple(kp), kd=tuple(kd)),
        joints=joint_names,
        desired_state=lambda time: reference,
    )


def system(algorithm, mechanism, *, parameters: Mapping, targets: Mapping):
    """Drake ports carry each device's state and effort; calculation stays coupled."""
    from pydrake.systems.framework import BasicVector, LeafSystem

    command = make_policy(algorithm, mechanism, parameters=parameters, targets=targets)

    class MechanismController(LeafSystem):
        def __init__(self):
            super().__init__()
            self.inputs = {}
            for name, indices in mechanism.indices.items():
                self.inputs[name] = self.DeclareVectorInputPort(
                    name + "/state", len(indices.q) + len(indices.v)
                )
                self.DeclareVectorOutputPort(
                    name + "/effort",
                    BasicVector(len(indices.u)),
                    lambda context, output, name=name: self.output(
                        context, output, name
                    ),
                )

        def output(self, context, output, name):
            n = mechanism.plant.num_positions()
            state = np.empty(n + mechanism.plant.num_velocities())
            for device, indices in mechanism.indices.items():
                observed = self.inputs[device].Eval(context)
                state[list(indices.q)] = observed[: len(indices.q)]
                state[n + np.asarray(indices.v)] = observed[len(indices.q) :]
            output.SetFromVector(
                command(state, context.get_time())[list(mechanism.indices[name].u)]
            )

    return MechanismController()
