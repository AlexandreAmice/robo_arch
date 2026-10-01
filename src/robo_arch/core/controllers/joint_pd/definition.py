"""Parameters for torque feedback, independent of simulator/native imports."""

from pydantic import Field, model_validator

from robo_arch.core.config.parameters import Parameters


class JointPdParameters(Parameters):
    """Kp in N m/rad and Kd in N m s/rad, in the selected robot's joint order."""

    kp: tuple[float, ...] = Field(min_length=1)
    kd: tuple[float, ...] = Field(min_length=1)

    @model_validator(mode="after")
    def gains(self) -> "JointPdParameters":
        if len(self.kp) != len(self.kd) or any(v < 0 for v in (*self.kp, *self.kd)):
            raise ValueError("PD gains must have equal lengths and be nonnegative")
        return self
