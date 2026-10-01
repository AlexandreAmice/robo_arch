"""Nominal iiwa 7 model identity without simulator imports."""

from robo_arch.core.config.declarations import RobotDefinition

BASE_FRAME = "iiwa_link_0"
TOOL_FRAME = "iiwa_link_ee"
JOINT_NAMES = tuple(f"iiwa_joint_{i}" for i in range(1, 8))
DEFAULT_POSITIONS = (0.0, 0.4, 0.0, -0.8, 0.0, 0.5, 0.0)


def describe() -> RobotDefinition:
    return RobotDefinition(
        base_frame=BASE_FRAME,
        joints=JOINT_NAMES,
        default_positions=DEFAULT_POSITIONS,
        supported_worlds=("drake", "isaac"),
    )
