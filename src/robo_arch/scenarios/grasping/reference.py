"""A repeatable approach, close, lift and release trajectory for a nominal box."""

import numpy as np
from pydrake.math import RotationMatrix
from pydrake.multibody.inverse_kinematics import InverseKinematics
from pydrake.solvers import Solve


def make_reference(model, mode="grasp"):
    """Solve nominal arm waypoints; no simulated object pose enters this reference."""
    plant = model.plant
    initial = np.r_[[-1.57, -1.57, 1.57, -1.57, -1.57, 0], [-0.045, 0.045]]
    frames = plant.GetFrameByName("body", model.robots["gripper"])
    points = [(0.4, 0, 0.35), (0.4, 0, 0.133), (0.4, 0, 0.28)]
    if mode == "push":
        points = [(0.30, 0, 0.35), (0.30, 0, 0.14), (0.50, 0, 0.14)]
    solutions = []
    for position in points:
        ik = InverseKinematics(plant)
        q = ik.q()
        target = np.asarray(position)
        ik.AddPositionConstraint(
            frames, [0, 0, 0], plant.world_frame(), target - 1e-5, target + 1e-5
        )
        ik.AddOrientationConstraint(
            plant.world_frame(),
            RotationMatrix.MakeXRotation(-np.pi / 2),
            frames,
            RotationMatrix(),
            1e-4,
        )
        ik.prog().AddBoundingBoxConstraint([-0.045, 0.045], [-0.045, 0.045], q[6:])
        ik.prog().AddQuadraticErrorCost(np.eye(8), initial, q)
        ik.prog().SetInitialGuess(q, initial)
        result = Solve(ik.prog())
        if not result.is_success():
            raise ValueError(f"Cannot reach nominal gripper waypoint {position}")
        initial = result.GetSolution(q)
        solutions.append(initial[:6])
    times = [0, 1, 3, 4, 6, 7, 8, 10]
    path = [
        solutions[0],
        solutions[0],
        solutions[1],
        solutions[1],
        solutions[2],
        solutions[2],
        solutions[2],
        solutions[2],
    ]

    def reference(time):
        index = min(np.searchsorted(times, time, side="right") - 1, len(times) - 2)
        fraction = np.clip(
            (time - times[index]) / (times[index + 1] - times[index]), 0, 1
        )
        blend = fraction**3 * (10 - 15 * fraction + 6 * fraction**2)
        rate = (
            30 * fraction**2 * (1 - fraction) ** 2 / (times[index + 1] - times[index])
        )
        delta = path[index + 1] - path[index]
        aperture = 0.09 if time < 3 or time >= 7 else 0.025
        if mode == "push":
            aperture = 0.0
        return path[index] + blend * delta, rate * delta, aperture

    return reference, np.r_[solutions[0], [-0.045, 0.045]]
