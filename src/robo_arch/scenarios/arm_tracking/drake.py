"""Native Drake assembly for the arm-tracking task."""

import numpy as np
from pydrake.geometry import Meshcat
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

from robo_arch.core.config.loading import RunConfiguration
from robo_arch.core.controllers import definition
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters
from robo_arch.core.worlds.drake.scene import DrakeScene, build_scene
from robo_arch.core.worlds.drake.visualization import add_visualization
from robo_arch.core.worlds.registry import Registry


def build_simulation(
    run: RunConfiguration,
    registry: Registry,
    parameters: JointTrackingParameters,
    *,
    desired_positions: tuple[float, ...],
    meshcat: Meshcat | None = None,
) -> tuple[Simulator, DrakeScene]:
    """Connect ideal joint measurements, a constant target, and effort control."""
    if len(run.robots) != 1:
        raise ValueError("Arm tracking requires exactly one actuated robot")
    builder = DiagramBuilder()
    scene = build_scene(builder, run, registry)
    robot = run.robots[0]
    joints = registry.robots[robot.model].joints
    model = scene.controller_models[robot.name]
    if (
        len(desired_positions) != len(joints)
        or not np.isfinite(desired_positions).all()
    ):
        raise ValueError("Desired positions must match the robot's joints")
    if not (
        np.all(desired_positions >= model.GetPositionLowerLimits())
        and np.all(desired_positions <= model.GetPositionUpperLimits())
    ):
        raise ValueError("Desired positions exceed the robot's joint limits")
    goal = builder.AddSystem(
        ConstantVectorSource([*desired_positions, *np.zeros(len(joints))])
    )
    ports = (
        definition(run.autonomy.controller)
        .IMPLEMENTATIONS["drake"]
        .load()(
            builder,
            scene,
            robot=robot.name,
            parameters=parameters,
            joints=joints,
        )
    )
    if set(ports) != {"desired_state"}:
        raise ValueError("Joint tracking task requires a desired_state reference port")
    builder.Connect(goal.get_output_port(), ports["desired_state"])
    if meshcat is not None:
        add_visualization(builder, scene, run.world_config.visualization, meshcat)
    simulator = Simulator(builder.Build())
    simulator.set_target_realtime_rate(run.world_config.target_realtime_rate)
    plant_context = scene.plant.GetMyMutableContextFromRoot(
        simulator.get_mutable_context()
    )
    for name, positions in scene.initial_positions.items():
        scene.plant.SetPositions(plant_context, scene.robots[name], positions)
    simulator.Initialize()
    return simulator, scene
