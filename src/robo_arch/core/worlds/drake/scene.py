"""Assemble fixed-base physical scenes with device-owned Drake factories."""

from dataclasses import dataclass
from importlib.resources import as_file, files

import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import RgbdSensor

from robo_arch.core.config.loading import Pose, RobotInstance, RunConfiguration
from robo_arch.core.worlds.registry import Registry, RobotDefinition


def _transform(pose: Pose) -> RigidTransform:
    return RigidTransform(RollPitchYaw(pose.rpy), pose.translation)


def base_pose(robot: RobotInstance) -> RigidTransform:
    result = RigidTransform()
    for pose in robot.poses:
        result = result @ _transform(pose)
    return result


def build_controller_model(
    robot: RobotInstance, definition: RobotDefinition
) -> MultibodyPlant:
    """Independent nominal dynamics, also usable when another world supplies state."""
    model = MultibodyPlant(0.0)
    instance = definition.implementations["drake"].load()(model, name=robot.name)
    model.WeldFrames(
        model.world_frame(),
        model.GetFrameByName(definition.base_frame, instance),
        base_pose(robot),
    )
    model.Finalize()
    return model


@dataclass
class DrakeScene:
    """Physical models and sensors added to a caller-owned DiagramBuilder.

    Controller models are independent of simulated state. Initial positions are
    applied by the caller after constructing the complete diagram's context.
    """

    plant: MultibodyPlant
    scene_graph: SceneGraph
    robots: dict[str, ModelInstanceIndex]
    cameras: dict[str, RgbdSensor]
    controller_models: dict[str, MultibodyPlant]
    initial_positions: dict[str, tuple[float, ...]]


def build_scene(
    builder: DiagramBuilder,
    run: RunConfiguration,
    registry: Registry,
) -> DrakeScene:
    """Add the physical scene; the caller supplies autonomy and builds the diagram.

    Objects are fixed fixtures. Controller models have the scene's base poses
    and gravity but contain only their robot's dynamics.
    """
    if run.world != "drake":
        raise ValueError(f"This scene builder cannot execute world {run.world}")

    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, run.time_step)
    robot_instances = {}
    controller_models = {}
    initial_positions = {}
    for robot in run.robots:
        definition = registry.robots[robot.model]
        add_robot = definition.implementations[run.world].load()
        instance = add_robot(plant, name=robot.name)
        X_WB = base_pose(robot)
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(definition.base_frame, instance),
            X_WB,
        )
        model = build_controller_model(robot, definition)
        positions = (
            definition.default_positions
            if robot.initial_positions is None
            else robot.initial_positions
        )
        if len(positions) != len(definition.joints) or not np.isfinite(positions).all():
            raise ValueError(f"Invalid initial positions for {robot.name}")
        if not (
            np.all(positions >= model.GetPositionLowerLimits())
            and np.all(positions <= model.GetPositionUpperLimits())
        ):
            raise ValueError(f"Initial positions exceed limits for {robot.name}")
        robot_instances[robot.name] = instance
        controller_models[robot.name] = model
        initial_positions[robot.name] = positions

    for obj in run.objects:
        definition = registry.objects[obj.model]
        parser = Parser(plant)
        parser.SetAutoRenaming(True)
        with as_file(files(definition.package).joinpath(definition.resource)) as path:
            (instance,) = parser.AddModels(str(path))
        plant.RenameModelInstance(instance, obj.name)
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(definition.base_frame, instance),
            _transform(obj.pose),
        )
    plant.Finalize()

    cameras = {}
    for sensor in run.sensors:
        robot_name, frame_name = sensor.parent.rsplit("/", 1)
        frame = plant.GetFrameByName(frame_name, robot_instances[robot_name])
        definition = registry.sensors[sensor.model]
        camera = definition.implementations[run.world].load()(
            builder,
            scene_graph,
            parent_frame_id=plant.GetBodyFrameIdOrThrow(frame.body().index()),
            X_PB=frame.GetFixedPoseInBodyFrame() @ _transform(sensor.pose),
            parameters=definition.parameter_schema.model_validate(sensor.parameters),
        )
        camera.set_name(sensor.name)
        cameras[sensor.name] = camera

    return DrakeScene(
        plant,
        scene_graph,
        robot_instances,
        cameras,
        controller_models,
        initial_positions,
    )
