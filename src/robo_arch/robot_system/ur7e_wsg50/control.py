"""Coupled nominal arm tracking with separately commanded WSG finger forces."""

import numpy as np
from pydrake.multibody.tree import MultibodyForces

from robo_arch.core.worlds.drake.scene import ControllerMechanism
from robo_arch.robots.wsg50.control import effort as gripper_effort


class Controller:
    """Use full mechanism dynamics, including current finger configuration.

    The arm acceleration target and selected gripper force jointly determine
    the nominal arm force through the gripper block of the mass matrix. Contacts
    remain external disturbances, never privileged simulator-model inputs.
    """

    def __init__(self, model: ControllerMechanism, reference):
        self.model = model
        self.reference = reference
        self.context = model.plant.CreateDefaultContext()
        self.arm = np.asarray(model.indices["arm"].v)
        self.gripper = np.asarray(model.indices["gripper"].v)
        if model.plant.num_positions() != 8 or model.plant.num_actuated_dofs() != 8:
            raise ValueError(
                "UR7e/WSG controller requires the declared eight-DOF mechanism"
            )
        self.limits = np.array(
            [
                model.plant.get_joint_actuator(i).effort_limit()
                for i in model.plant.GetJointActuatorIndices()
            ]
        )

    def __call__(self, state: np.ndarray, time: float) -> np.ndarray:
        plant = self.model.plant
        plant.SetPositionsAndVelocities(self.context, state)
        q, v = state[:8], state[8:]
        target, rate, aperture = self.reference(time)
        acceleration = 150.0 * (target - q[self.arm]) + 28.0 * (rate - v[self.arm])
        ug = gripper_effort(q[self.gripper], v[self.gripper], aperture)
        mass = plant.CalcMassMatrix(self.context)
        forces = MultibodyForces(plant)
        plant.CalcForceElementsContribution(self.context, forces)
        zero = np.zeros(8)
        bias = plant.CalcInverseDynamics(self.context, zero, forces)
        a, g = self.arm, self.gripper
        ag = np.linalg.solve(
            mass[np.ix_(g, g)], ug - bias[g] - mass[np.ix_(g, a)] @ acceleration
        )
        effort = np.empty(8)
        effort[a] = (
            mass[np.ix_(a, a)] @ acceleration + mass[np.ix_(a, g)] @ ag + bias[a]
        )
        effort[g] = ug
        return np.clip(effort, -self.limits, self.limits)
