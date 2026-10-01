"""Assemble fixed-base physical scenes from declared assets and mounts."""

from dataclasses import dataclass
from importlib.resources import as_file, files

import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.sensors import RgbdSensor

from robo_arch.core.config.declarations import (
    Pose,
    RobotDefinition,
    SceneConfiguration,
    SensorInstance,
)
from robo_arch.core.worlds.assembly import PlacedRobot, resolve_devices
from robo_arch.core.worlds.devices import (
    DeviceDefinitions,
    load_definitions,
    load_device_module,
)
from robo_arch.core.worlds.drake.config import DrakePhysics, DrakeWorld
from robo_arch.core.worlds.drake.models import add_robot
from robo_arch.core.worlds.drake.sensors import add_sensor_body


def _transform(pose: Pose) -> RigidTransform:
    return RigidTransform(RollPitchYaw(pose.rpy), pose.translation)


def base_pose(robot: PlacedRobot) -> RigidTransform:
    result = RigidTransform()
    for pose in robot.poses:
        result = result @ _transform(pose)
    return result


def build_controller_model(
    robot: PlacedRobot,
    definition: RobotDefinition,
    *,
    sensors: tuple[SensorInstance, ...] = (),
    definitions: DeviceDefinitions | None = None,
) -> MultibodyPlant:
    """Independent nominal dynamics, also usable when another world supplies state."""
    if sensors and definitions is None:
        raise ValueError("Mounted sensor models require their definitions")
    model = MultibodyPlant(0.0)
    instance = add_robot(model, definition, name=robot.name)
    model.WeldFrames(
        model.world_frame(),
        model.GetFrameByName(definition.base_frame, instance),
        base_pose(robot),
    )
    for sensor in sensors:
        parent_name, frame_name = sensor.parent.rsplit("/", 1)
        if parent_name == robot.name:
            add_sensor_body(
                model,
                sensor,
                definitions.sensors[sensor.model],
                model.GetFrameByName(frame_name, instance),
                _transform(sensor.pose),
            )
    model.Finalize()
    return model


@dataclass
class DrakeScene:
    """Physical models and sensors added to a caller-owned DiagramBuilder.

    Controller models are independent of simulated state. Initial positions are
    applied by the caller after constructing the complete diagram's context.
    """

    definitions: DeviceDefinitions
    plant: MultibodyPlant
    scene_graph: SceneGraph
    robots: dict[str, ModelInstanceIndex]
    cameras: dict[str, RgbdSensor]
    wrenches: dict[str, LeafSystem]
    sensor_instances: dict[str, ModelInstanceIndex]
    controller_models: dict[str, MultibodyPlant]
    initial_positions: dict[str, tuple[float, ...]]


def add_plant(
    builder: DiagramBuilder, physics: DrakePhysics
) -> tuple[MultibodyPlant, SceneGraph]:
    """Apply native numerical settings before loading or finalizing models."""
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, physics.time_step)
    ApplyMultibodyPlantConfig(
        MultibodyPlantConfig(
            time_step=physics.time_step,
            contact_model=physics.contact_model,
            discrete_contact_approximation=physics.discrete_contact_approximation,
            sap_near_rigid_threshold=physics.sap_near_rigid_threshold,
        ),
        plant,
    )
    return plant, scene_graph


def build_scene(
    scene: SceneConfiguration,
    config: DrakeWorld,
    *,
    builder: DiagramBuilder,
) -> DrakeScene:
    """Add the physical scene; the caller supplies autonomy and builds the diagram.

    Objects are fixed fixtures. Controller models have the scene's base poses
    and gravity and contain their robot plus its mounted devices.
    """
    if not isinstance(config, DrakeWorld):
        raise ValueError("Drake scene construction requires DrakeWorld")
    definitions = load_definitions(scene, config.type)

    devices = resolve_devices(scene)
    plant, scene_graph = add_plant(builder, config.physics)
    robot_instances = {}
    controller_models = {}
    initial_positions = {}
    for robot in devices.robots:
        definition = definitions.robots[robot.model]
        instance = add_robot(plant, definition, name=robot.name)
        X_WB = base_pose(robot)
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(definition.base_frame, instance),
            X_WB,
        )
        model = build_controller_model(
            robot, definition, sensors=devices.sensors, definitions=definitions
        )
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

    for obj in scene.objects:
        definition = definitions.objects[obj.model]
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
    sensor_instances = {}
    for sensor in devices.sensors:
        robot_name, frame_name = sensor.parent.rsplit("/", 1)
        sensor_instances[sensor.name] = add_sensor_body(
            plant,
            sensor,
            definitions.sensors[sensor.model],
            plant.GetFrameByName(frame_name, robot_instances[robot_name]),
            _transform(sensor.pose),
        )
    plant.Finalize()

    cameras = {}
    wrenches = {}
    for sensor in devices.sensors if scene.sensors_enabled else ():
        definition = definitions.sensors[sensor.model]
        adapter = load_device_module("sensors", sensor.model, "drake")
        instance = sensor_instances[sensor.name]
        if definition.kind == "wrench":
            observation = adapter.add_to_builder(builder, plant, instance=instance)
            wrenches[sensor.name] = observation
        elif definition.kind == "camera":
            frame = plant.GetFrameByName(definition.measurement_frame, instance)
            observation = adapter.add_to_builder(
                builder,
                scene_graph,
                parent_frame_id=plant.GetBodyFrameIdOrThrow(frame.body().index()),
                X_PB=frame.GetFixedPoseInBodyFrame(),
                parameters=definition.parameter_schema.model_validate(
                    sensor.parameters
                ),
            )
            cameras[sensor.name] = observation
        else:
            raise ValueError(f"Unsupported sensor kind {definition.kind}")
        observation.set_name(sensor.name)

    return DrakeScene(
        definitions,
        plant,
        scene_graph,
        robot_instances,
        cameras,
        wrenches,
        sensor_instances,
        controller_models,
        initial_positions,
    )
