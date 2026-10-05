"""Construct arm-tracking effort callbacks for an existing Isaac scene."""

from collections.abc import Callable

import numpy as np

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.worlds.isaac.scene import IsaacScene


def configure(
    scene: IsaacScene, run: RunConfiguration, parameters: dict, tasks: dict
) -> dict[str, Callable[[np.ndarray, float], np.ndarray]]:
    """Use independent nominal dynamics and the selected shared controller."""
    from robo_arch.core.controllers.mechanism import make_policy
    from robo_arch.core.controllers.selection import select_controller
    from robo_arch.core.worlds.drake.scene import build_mechanism_model

    select_controller(run.world_config, run.autonomy.controller)
    commands = {}
    for mechanism in scene.devices.mechanisms:
        model = build_mechanism_model(
            mechanism, scene.definitions, sensors=scene.devices.sensors
        )
        commands[mechanism.root] = make_policy(
            run.autonomy.controller,
            model,
            parameters={name: parameters[name] for name in model.indices},
            targets={name: tasks[name].target for name in model.indices},
        )
    return commands
