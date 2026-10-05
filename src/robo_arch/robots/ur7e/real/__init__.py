"""UR joint identity and trajectory capability mapping for the ROS driver."""

from robo_arch.core.config.loading import load_robot
from robo_arch.core.contracts.commands import CommandCapabilities, CommandKind
from robo_arch.core.worlds.assembly import PlacedRobot
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.core.worlds.real.trajectory import TrajectoryTransport

CAPABILITIES = CommandCapabilities(frozenset({CommandKind.JOINT_POSITION_TRAJECTORY}))


def validate_selection(robot: PlacedRobot, command: CommandKind) -> tuple[str, ...]:
    """Validate installation/calibration before importing or connecting ROS."""
    CAPABILITIES.require(command)
    if robot.model != "ur7e" or robot.parent is not None:
        raise ValueError("This driver maps one root UR7e arm")
    if robot.binding is None or not robot.binding.endpoint:
        raise ValueError(
            "UR deployment needs an installation identity and ROS namespace"
        )
    calibration = robot.calibration
    if calibration is None:
        raise ValueError(
            "Select an explicit nominal, synthetic or measured calibration"
        )
    if (
        calibration.identity != robot.binding.identity
        or calibration.mounting_revision != robot.mounting_revision
    ):
        raise ValueError("UR calibration identity/revision differs from installation")
    if calibration.parent_identity is not None or calibration.parent_frame != "world":
        raise ValueError("Root UR calibration must identify the world parent frame")
    definition = load_robot("package://robo_arch/robots/ur7e/robot.yaml")
    if calibration.child_frame != definition.base_frame:
        raise ValueError("UR mount calibration must identify the declared base frame")
    return definition.joints


class UrTrajectoryAdapter(TrajectoryTransport):
    """UR radians/radians-per-second mapping; no effort command capability.

    ``robot.name`` is the scene instance path; ``binding.identity`` identifies the
    installation, ``physical_id`` is optional, and ``endpoint`` is its namespace.
    The selected mounting transform is metadata; q/v are joint coordinates.
    Default ROS effort values can represent motor current and are never read.
    """

    def __init__(
        self, robot: PlacedRobot, config: RealWorld, command: CommandKind
    ) -> None:
        names = validate_selection(robot, command)
        self.instance = robot
        super().__init__(
            names=names,
            namespace=robot.binding.endpoint,
            controller=config.trajectory_controller,
            max_age=config.observation_max_age,
            timeout=config.operation_timeout,
        )


def build_adapter(
    robot: PlacedRobot, config: RealWorld, command: CommandKind
) -> UrTrajectoryAdapter:
    """Conventional device entry point used by real-world assembly."""
    return UrTrajectoryAdapter(robot, config, command)
