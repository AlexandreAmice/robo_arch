"""Construct arm-tracking effort callbacks for an existing Isaac scene."""

from collections.abc import Callable

import numpy as np

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.isaac.scene import IsaacScene
from robo_arch.scenarios.arm_tracking.control import make_policy


def configure(
    scene: IsaacScene, run: RunConfiguration, parameters: dict, tasks: dict
) -> dict[str, Callable[[np.ndarray, float], np.ndarray]]:
    """Use independent nominal dynamics and the selected shared controller."""
    import warnings

    from robo_arch.core.worlds.drake.scene import build_controller_model

    transfers = (
        " GPU state and effort transfer each step."
        if run.world_config.physics.device == "cuda:0"
        else ""
    )
    warnings.warn(
        f"{run.autonomy.controller} uses scalar CPU control in Isaac." + transfers,
        RuntimeWarning,
        stacklevel=2,
    )
    devices = resolve_devices(run.scene)
    definitions = scene.definitions
    commands = {}
    for robot in devices.robots:
        definition = definitions.robots[robot.model]
        model = build_controller_model(
            robot, definition, sensors=devices.sensors, definitions=definitions
        )
        target = np.asarray(tasks[robot.name].target)
        if target.shape != (model.num_positions(),) or not (
            np.all(target >= model.GetPositionLowerLimits())
            and np.all(target <= model.GetPositionUpperLimits())
        ):
            raise ValueError(f"Task target must match {robot.name}'s joints and limits")
        reference = np.r_[target, np.zeros_like(target)]
        commands[robot.name] = make_policy(
            run,
            model=model,
            parameters=parameters[robot.name],
            joints=definition.joints,
            desired_state=lambda time, reference=reference: reference,
        )
    return commands
