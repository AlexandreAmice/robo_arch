"""Assemble fixed-base physical scenes from declared assets and mounts."""

from dataclasses import dataclass
from importlib.resources import as_file, files

import numpy as np
from pydrake.geometry import (
    AddContactMaterial,
    AddRigidHydroelasticProperties,
    Box,
    HalfSpace,
    ProximityProperties,
    SceneGraph,
)
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import (
    AddMultibodyPlantSceneGraph,
    ApplyMultibodyPlantConfig,
    CoulombFriction,
    MultibodyPlant,
    MultibodyPlantConfig,
)
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import DiagramBuilder, LeafSystem
from pydrake.systems.sensors import RgbdSensor

from robo_arch.core.config.declarations import (
    RobotDefinition,
    SceneConfiguration,
    SensorInstance,
)
from robo_arch.core.worlds.assembly import (
    Mechanism,
    PlacedRobot,
    attachment_pose,
    pose_transform,
    resolve_devices,
)
from robo_arch.core.worlds.devices import (
    DeviceDefinitions,
    load_definitions,
    load_device_module,
)
from robo_arch.core.worlds.drake.config import DrakePhysics, DrakeWorld
from robo_arch.core.worlds.drake.models import add_robot
from robo_arch.core.worlds.drake.sensors import add_sensor_body
from robo_arch.core.worlds.urdf import mount_to_base


@dataclass(frozen=True)
class JointIndices:
    """Indices into the connected plant's q, v and actuator vectors."""

    q: tuple[int, ...]
    v: tuple[int, ...]
    u: tuple[int, ...]


@dataclass(frozen=True)
class ControllerMechanism:
    """Independent nominal model with explicit per-device vector ownership."""

    plant: MultibodyPlant
    robots: dict[str, ModelInstanceIndex]
    indices: dict[str, JointIndices]


def _add_mechanism(plant, mechanism, definitions, sensors):
    instances = {}
    for robot in mechanism.robots:
        definition = definitions.robots[robot.model]
        mount_to_base(robot, definitions)  # Validate the shared rigid-mount contract.
        instance = add_robot(plant, definition, name=robot.name)
        if robot.parent:
            parent, frame = robot.parent.rsplit("/", 1)
            parent_frame = plant.GetFrameByName(frame, instances[parent])
        else:
            parent_frame = plant.world_frame()
        child_frame = robot.mount_frame or definition.base_frame
        if robot.calibration and robot.calibration.child_frame != child_frame:
            raise ValueError(f"Calibration child frame differs for {robot.name}")
        plant.WeldFrames(
            parent_frame,
            plant.GetFrameByName(child_frame, instance),
            attachment_pose(robot),
        )
        instances[robot.name] = instance
    sensor_instances = {}
    for sensor in sensors:
        parent, frame = sensor.parent.rsplit("/", 1)
        if parent in instances:
            definition = definitions.sensors[sensor.model]
            if (
                sensor.calibration
                and sensor.calibration.child_frame != definition.base_frame
            ):
                raise ValueError(f"Calibration child frame differs for {sensor.name}")
            sensor_instances[sensor.name] = add_sensor_body(
                plant,
                sensor,
                definition,
                plant.GetFrameByName(frame, instances[parent]),
                pose_transform(
                    sensor.calibration.pose if sensor.calibration else sensor.pose
                ),
            )
    return instances, sensor_instances


def build_mechanism_model(
    mechanism: Mechanism,
    definitions: DeviceDefinitions,
    *,
    sensors: tuple[SensorInstance, ...] = (),
) -> ControllerMechanism:
    """Build full connected nominal dynamics, including actuated tool state."""
    plant = MultibodyPlant(0.0)
    instances, _ = _add_mechanism(plant, mechanism, definitions, sensors)
    plant.Finalize()
    indices = {}
    actuator_indices = list(plant.GetJointActuatorIndices())
    for robot in mechanism.robots:
        instance = instances[robot.name]
        joints = [
            plant.GetJointByName(name, instance)
            for name in definitions.robots[robot.model].joints
        ]
        if any(j.num_positions() != 1 or j.num_velocities() != 1 for j in joints):
            raise ValueError("Declared device joints must be scalar")
        indices[robot.name] = JointIndices(
            tuple(j.position_start() for j in joints),
            tuple(j.velocity_start() for j in joints),
            tuple(
                i
                for i, index in enumerate(actuator_indices)
                if plant.get_joint_actuator(index).model_instance() == instance
            ),
        )
    return ControllerMechanism(plant, instances, indices)


def build_controller_model(
    robot: PlacedRobot,
    definition: RobotDefinition,
    *,
    sensors: tuple[SensorInstance, ...] = (),
    definitions: DeviceDefinitions | None = None,
) -> MultibodyPlant:
    """Standalone compatibility helper; attached tools require the full mechanism."""
    if robot.parent:
        raise ValueError("Attached robot requires build_mechanism_model")
    if definitions is None:
        if sensors:
            raise ValueError("Mounted sensor models require their definitions")
        definitions = DeviceDefinitions(
            robots={robot.model: definition}, sensors={}, objects={}
        )
    return build_mechanism_model(
        Mechanism(robot.name, (robot,)), definitions, sensors=sensors
    ).plant


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
    mechanism_models: dict[str, ControllerMechanism]
    objects: dict[str, ModelInstanceIndex]
    configuration: SceneConfiguration


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


def add_ground(plant: MultibodyPlant) -> None:
    """Add an infinite collidable floor at z=0 with a 10 m square visible top.

    The rigid half-space also supports compliant hydroelastic contact. Its own
    material does not change any device or object contact properties.
    """
    material = ProximityProperties()
    AddContactMaterial(1.0, 1e5, CoulombFriction(0.8, 0.6), material)
    AddRigidHydroelasticProperties(material)
    plant.RegisterCollisionGeometry(
        plant.world_body(), RigidTransform(), HalfSpace(), "ground_collision", material
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        RigidTransform([0, 0, -0.05]),
        Box(10, 10, 0.1),
        "ground_visual",
        [0.55, 0.57, 0.60, 1.0],
    )


def build_scene(
    scene: SceneConfiguration,
    config: DrakeWorld,
    *,
    builder: DiagramBuilder,
) -> DrakeScene:
    """Add the physical scene; the caller supplies autonomy and builds the diagram.

    Objects are fixed or free as declared. Independent controller models contain
    the full connected mechanism, including actuated tools and mounted sensors.
    """
    if not isinstance(config, DrakeWorld):
        raise ValueError("Drake scene construction requires DrakeWorld")
    definitions = load_definitions(scene, config.type)

    devices = resolve_devices(scene)
    plant, scene_graph = add_plant(builder, config.physics)
    if config.ground:
        add_ground(plant)
    robot_instances = {}
    controller_models = {}
    initial_positions = {}
    sensor_instances = {}
    mechanism_models = {}
    for mechanism in devices.mechanisms:
        instances, mounted_sensors = _add_mechanism(
            plant, mechanism, definitions, devices.sensors
        )
        robot_instances.update(instances)
        sensor_instances.update(mounted_sensors)
        nominal = build_mechanism_model(mechanism, definitions, sensors=devices.sensors)
        mechanism_models[mechanism.root] = nominal
        for robot in mechanism.robots:
            definition = definitions.robots[robot.model]
            positions = (
                definition.default_positions
                if robot.initial_positions is None
                else robot.initial_positions
            )
            indices = list(nominal.indices[robot.name].q)
            lower = nominal.plant.GetPositionLowerLimits()[indices]
            upper = nominal.plant.GetPositionUpperLimits()[indices]
            if (
                len(positions) != len(indices)
                or not np.isfinite(positions).all()
                or not np.all((positions >= lower) & (positions <= upper))
            ):
                raise ValueError(
                    f"Invalid initial positions or limits for {robot.name}"
                )
            controller_models[robot.name] = nominal.plant
            initial_positions[robot.name] = positions
    object_instances = {}

    for obj in scene.objects:
        definition = definitions.objects[obj.model]
        parser = Parser(plant)
        parser.SetAutoRenaming(True)
        with as_file(files(definition.package).joinpath(definition.resource)) as path:
            (instance,) = parser.AddModels(str(path))
        plant.RenameModelInstance(instance, obj.name)
        object_instances[obj.name] = instance
        if obj.motion == "fixed":
            plant.WeldFrames(
                plant.world_frame(),
                plant.GetFrameByName(definition.base_frame, instance),
                pose_transform(obj.pose),
            )
        else:
            plant.SetDefaultFloatingBaseBodyPose(
                plant.GetBodyByName(definition.base_frame, instance),
                pose_transform(obj.pose),
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
        mechanism_models,
        object_instances,
        scene,
    )


def initialize_objects(scene: DrakeScene, context) -> None:
    """Restore declared free-body pose and world-expressed origin velocity."""
    from pydrake.multibody.math import SpatialVelocity

    for obj in scene.configuration.objects:
        if obj.motion == "free":
            definition = scene.definitions.objects[obj.model]
            body = scene.plant.GetBodyByName(
                definition.base_frame, scene.objects[obj.name]
            )
            scene.plant.SetFreeBodyPose(context, body, pose_transform(obj.pose))
            scene.plant.SetFreeBodySpatialVelocity(
                context,
                body,
                SpatialVelocity(w=obj.angular_velocity, v=obj.linear_velocity),
            )
