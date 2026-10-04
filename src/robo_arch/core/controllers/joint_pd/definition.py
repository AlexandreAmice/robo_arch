"""Parameters for torque feedback, independent of simulator/native imports."""

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters


class JointPdParameters(Parameters):
    """Torque-feedback gains in the selected robot's revolute-joint order.

    :param kp: Finite nonnegative proportional gains, in N m/rad.
    :param kd: Finite nonnegative derivative gains, in N m s/rad.

    Both tuples must be nonempty and equally long; invalid values raise
    ValidationError. Matching the robot's joint count/order is the adapter's
    responsibility. These differ from joint_tracking's acceleration gains.
    """

    kp: tuple[float, ...] = Field(min_length=1)
    kd: tuple[float, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def gains(self) -> "JointPdParameters":
        if len(self.kp) != len(self.kd) or any(v < 0 for v in (*self.kp, *self.kd)):
            raise ValueError("PD gains must have equal lengths and be nonnegative")
        return self
