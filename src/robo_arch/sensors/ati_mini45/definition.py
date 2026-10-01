"""Mini45-R declaration; nominal geometry, ideal six-axis sensing."""

from robo_arch.core.config.declarations import SensorDefinition
from robo_arch.core.config.parameters import Parameters


class Mini45Parameters(Parameters):
    """No filtering, electronic noise, bias or software gravity compensation."""


def describe() -> SensorDefinition:
    return SensorDefinition(
        parameter_schema=Mini45Parameters,
        supported_worlds=("drake", "isaac"),
        physical_worlds=("drake", "isaac"),
        kind="wrench",
        base_frame="mount",
        measurement_frame="sensing",
        resource="assets/model.urdf",
    )
