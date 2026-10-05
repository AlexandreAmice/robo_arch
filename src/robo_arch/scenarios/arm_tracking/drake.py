"""Wire arm-tracking references and autonomy into an existing Drake scene."""

from pydrake.systems.framework import DiagramBuilder

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.controllers.selection import select_controller
from robo_arch.core.worlds.drake.scene import DrakeScene


def configure(
    builder: DiagramBuilder,
    scene: DrakeScene,
    run: RunConfiguration,
    parameters: dict,
    *,
    desired_positions: dict[str, tuple[float, ...]],
) -> None:
    from robo_arch.core.controllers.mechanism import system

    select_controller(run.world_config, run.autonomy.controller)
    for root, mechanism in scene.mechanism_models.items():
        names = mechanism.indices
        controller = builder.AddSystem(
            system(
                run.autonomy.controller,
                mechanism,
                parameters={name: parameters[name] for name in names},
                targets={name: desired_positions[name] for name in names},
            )
        )
        controller.set_name(root + "/" + run.autonomy.controller)
        for name in names:
            builder.Connect(
                scene.plant.get_state_output_port(scene.robots[name]),
                controller.GetInputPort(name + "/state"),
            )
            builder.Connect(
                controller.GetOutputPort(name + "/effort"),
                scene.plant.get_actuation_input_port(scene.robots[name]),
            )
