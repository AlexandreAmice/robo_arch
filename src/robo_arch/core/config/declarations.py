"""Loaded configuration records and reusable device metadata.

The loader turns YAML instance keys and system references into these records.
Device definitions describe reusable models, not live instances. Importing these
declarations does not load YAML or simulator SDKs.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Annotated, Literal

from pydantic import (
    Field,
    JsonValue,
    StringConstraints,
    field_validator,
    model_validator,
)

from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.resources import validate_package_reference
from robo_arch.core.config.schema import Schema
from robo_arch.core.config.worlds import WorldConfiguration

Name = Annotated[str, StringConstraints(pattern=r"^[A-Za-z][A-Za-z0-9_]*$")]


class Pose(Schema):
    """Placement of a child frame in its parent frame.

    :param translation: Child origin expressed in the parent, in metres (x, y, z).
    :param rpy: Fixed-axis roll, pitch and yaw in radians; the rotation is
        Rz(yaw) Ry(pitch) Rx(roll). Both tuples default to zero.

    This declaration performs no transform computation and imports no SDK.
    """

    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)


class TaskSelection(Schema):
    """Select a scenario's evaluation task.

    :param type: Scenario-defined task identifier, not an import path.
    :param parameters: JSON-compatible settings validated by the evaluator.
        The mapping defaults to empty and must be treated as read-only.
    """

    type: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class AutonomySelection(Schema):
    """Select autonomy supported by the scenario's Python wiring.

    :param controller: Scenario-defined controller identifier, not a factory path.
    :param parameters: JSON-compatible settings validated by the autonomy owner.
        This mapping does not describe an executable computation graph.
    """

    controller: Name
    parameters: dict[str, JsonValue] = Field(default_factory=dict)


class DeviceBinding(Schema):
    """Installation identity, separate from a serial number or transport endpoint."""

    identity: str = Field(min_length=1)
    physical_id: str | None = None
    endpoint: str | None = None


class CalibrationProfile(Schema):
    """One explicitly selected effective mount, in metres and fixed-axis radians.

    Synthetic profiles exercise identity checks without claiming measurements.
    The selected transform replaces the nominal mount; it is not a hidden offset.
    """

    identity: str
    parent_identity: str | None = None
    parent_frame: str
    child_frame: str
    mounting_revision: str
    kind: Literal["nominal", "synthetic", "measured"]
    pose: Pose


@dataclass(frozen=True, kw_only=True)
class RobotInstance:
    """One robot or actuated tool in a containing system.

    :param name: Local instance name; world assembly adds ancestor namespaces.
    :param model: Model package identifier under ``robots``.
    :param pose: Robot base frame relative to the containing system.
    :param initial_positions: Joint positions in the model's declared order,
        or None to use its defaults. Units follow the joint type (rad or m).

    This frozen record performs no model lookup or dimension validation.
    """

    name: str
    model: str
    pose: Pose
    initial_positions: tuple[float, ...] | None
    parent: str | None = None
    mount_frame: str | None = None
    binding: DeviceBinding | None = None
    calibration: CalibrationProfile | None = None
    mounting_revision: str | None = None


@dataclass(frozen=True, kw_only=True)
class SensorInstance:
    """One mounted sensor, whether or not observations are enabled.

    :param name: Local sensor instance name.
    :param model: Model package identifier under ``sensors``.
    :param parent: Local ``robot/body`` attachment, qualified by world assembly.
    :param pose: Sensor base frame relative to that parent body.
    :param parameters: Sensor-specific settings, validated when definitions load.
        Frozen records do not freeze this mapping; treat it as read-only.
    """

    name: str
    model: str
    parent: str
    pose: Pose
    parameters: dict[str, JsonValue]
    binding: DeviceBinding | None = None
    calibration: CalibrationProfile | None = None
    mounting_revision: str | None = None


@dataclass(frozen=True, kw_only=True)
class RobotSystem:
    """Physical composition with local instance names and relative placements.

    The root has an empty name and a world-relative pose. Child-system poses are
    relative to their containing system; sensor poses are relative to their parent
    robot/body. Each inclusion loads independent parameter dictionaries.

    :param name: Local child-system name; empty for the root.
    :param source: Resolved source YAML file for resource/provenance reporting.
    :param pose: Placement in the containing system, or world for the root.
    :param robots: Local robot/tool instances, in declaration order.
    :param sensors: Local sensor instances, retained when observations are off.
    :param systems: Nested physical compositions, each with independent identity.
    """

    name: str
    source: Path
    pose: Pose
    robots: tuple[RobotInstance, ...] = ()
    sensors: tuple[SensorInstance, ...] = ()
    systems: tuple["RobotSystem", ...] = ()


@dataclass(frozen=True, kw_only=True)
class ObjectInstance:
    """A named scene object, currently constructed as a fixed fixture.

    :param name: Scene-wide name, distinct from resolved device names.
    :param model: Model package identifier under ``objects``.
    :param pose: Object base frame relative to world; metres and radians.
    """

    name: str
    model: str
    pose: Pose
    motion: Literal["fixed", "free"] = "fixed"
    # Velocity at the object's declared base origin, expressed in world.
    angular_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)
    linear_velocity: tuple[float, float, float] = (0.0, 0.0, 0.0)


@dataclass(frozen=True, kw_only=True)
class SceneConfiguration:
    """Physical instances and observation selection, independent of a task.

    :param robot_system: Root physical composition.
    :param sensors_enabled: Enable observations. False still retains sensor
        bodies, mounts, inertia and collisions in physical assembly.
    :param objects: World-placed fixed objects, in declaration order.

    Records are shared with the originating run, not copied or instantiated here.
    """

    robot_system: RobotSystem
    sensors_enabled: bool
    objects: tuple[ObjectInstance, ...]


@dataclass(frozen=True, kw_only=True)
class RunConfiguration:
    """Validated selections and names; device compatibility is checked at assembly.

    The robot system retains its hierarchy and local names. World assembly
    resolves device names, sensor parents and robot pose chains. Objects are fixed
    scene fixtures in this first runtime. Parameters remain caller-owned; do not
    mutate after loading.

    :param source: Resolved scenario YAML file.
    :param world_config: Validated native world settings, without SDK objects.
    :param duration: Requested simulation duration in seconds.
    :param robot_system: Root composition with a world-relative pose.
    :param sensors_enabled: Whether to construct observation outputs.
    :param objects: World-placed fixed fixtures.
    :param task: Evaluation selection, validated further by the scenario.
    :param autonomy: Controller selection, validated further by scenario wiring.
    :param world_source: Resolved external world profile, or None for inline settings.

    Constructing this dataclass directly does not validate YAML or model support;
    use :func:`robo_arch.core.config.loading.load_run` at the configuration boundary.
    """

    source: Path
    world_config: WorldConfiguration
    duration: float
    robot_system: RobotSystem
    sensors_enabled: bool
    objects: tuple[ObjectInstance, ...]
    task: TaskSelection
    autonomy: AutonomySelection
    world_source: Path | None = None

    @property
    def scene(self) -> SceneConfiguration:
        """Physical view sharing this run's system and object records."""
        return SceneConfiguration(
            robot_system=self.robot_system,
            sensors_enabled=self.sensors_enabled,
            objects=self.objects,
        )

    @property
    def world(self) -> str:
        """World discriminator: ``drake``, ``isaac`` or ``real``."""
        return self.world_config.type

    @property
    def time_step(self) -> float:
        """Physics step in seconds; raises ValueError for the real world."""
        if self.world_config.type == "real":
            raise ValueError("The real world has no simulated physics time step")
        return self.world_config.physics.time_step

    @property
    def resources(self) -> tuple[Path, ...]:
        """Unique scenario, profile and system YAML paths in traversal order.

        Model assets and their dependent meshes are not included in this tuple.
        """
        paths = [self.source]
        if self.world_source is not None:
            paths.append(self.world_source)

        def visit(system: RobotSystem) -> None:
            paths.append(system.source)
            for child in system.systems:
                visit(child)

        visit(self.robot_system)
        return tuple(dict.fromkeys(paths))


class RobotDefinition(Schema):
    """Physical asset and joint/frame conventions; no executable factory.

    :param asset: ``package://robo_arch/...`` model asset reference. URI syntax is
        validated here; existence and format support are checked by world loading.
    :param base_frame: Base frame name within the model asset.
    :param joints: Nonempty, unique joint names defining the control vector order.
    :param default_positions: Finite positions in that order (rad or m by joint
        type); exactly one per joint. Invalid declarations raise ValidationError.
    """

    asset: str
    base_frame: str
    joints: tuple[str, ...]
    default_positions: tuple[float, ...]

    @field_validator("asset")
    @classmethod
    def _asset_reference(cls, value: str) -> str:
        return validate_package_reference(value)

    @model_validator(mode="after")
    def _joint_defaults(self) -> "RobotDefinition":
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise ValueError("Robot joints must be nonempty and unique")
        if len(self.default_positions) != len(self.joints):
            raise ValueError(
                "Robot default positions must match the declared joint order"
            )
        return self


@dataclass(frozen=True, kw_only=True)
class SensorDefinition:
    """Reusable sensor metadata returned by its owner's ``describe()`` function.

    :param parameter_schema: SDK-independent schema for instance parameters.
    :param supported_worlds: Worlds supporting observations.
    :param physical_worlds: Worlds supporting the sensor body and mounting.
    :param kind: Observation kind used by world assembly, e.g. ``camera``.
    :param base_frame: Sensor model's mounting frame.
    :param measurement_frame: Model frame in which measurements are defined.
    :param resource: Asset name relative to the owning sensor package.

    Physical support is required even when observations are disabled. These
    declarations contain no live device state and do not validate assets.
    """

    parameter_schema: type[Parameters]
    supported_worlds: tuple[str, ...]
    physical_worlds: tuple[str, ...] = ()
    kind: str = "camera"
    base_frame: str = "mount"
    measurement_frame: str = "depth_optical"
    resource: str = "assets/model.urdf"


@dataclass(frozen=True, kw_only=True)
class ObjectDefinition:
    """Reusable fixed-object metadata; importing it does not load a world.

    :param package: Importable owning package used to resolve the asset.
    :param resource: Asset name relative to that package.
    :param base_frame: Frame placed by the scene's object pose.
    :param supported_worlds: Worlds whose loaders support this declaration.

    Asset parsing and physical support checks belong to the selected world.
    """

    package: str
    resource: str
    base_frame: str
    supported_worlds: tuple[str, ...]
