"""Nominal UR7e frames and joint ordering, independent of simulation SDKs."""

from math import pi

from robo_arch.core.worlds.registry import FactoryReference, RobotDefinition

BASE_FRAME = "base_link"
TOOL_FRAME = "tool0"
JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)
# Radians, in JOINT_NAMES order; a bent arm for the tracking example.
DEFAULT_POSITIONS = (0.0, -pi / 2, pi / 2, -pi / 2, -pi / 2, 0.0)

DEFINITION = RobotDefinition(
    base_frame=BASE_FRAME,
    joints=JOINT_NAMES,
    default_positions=DEFAULT_POSITIONS,
    implementations={
        "isaac": FactoryReference(
            module="robo_arch.robots.ur7e.isaac", attribute="add_to_stage"
        ),
        "drake": FactoryReference(
            module="robo_arch.robots.ur7e.drake", attribute="add_to_plant"
        ),
    },
)
