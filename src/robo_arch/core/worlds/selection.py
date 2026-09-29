"""Check device selections before importing a simulator SDK."""

from robo_arch.core.config.loading import RunConfiguration
from robo_arch.core.worlds.registry import Registry


def validate_devices(run: RunConfiguration, registry: Registry) -> None:
    for robot in run.robots:
        if robot.model not in registry.robots:
            raise ValueError(f"Unknown robot model: {robot.model}")
        if run.world not in registry.robots[robot.model].implementations:
            raise ValueError(f"Robot {robot.name} has no {run.world} implementation")
    for sensor in run.sensors:
        if sensor.model not in registry.sensors:
            raise ValueError(f"Unknown sensor model: {sensor.model}")
        definition = registry.sensors[sensor.model]
        if run.world not in definition.implementations:
            raise ValueError(f"Sensor {sensor.name} has no {run.world} implementation")
        definition.parameter_schema.model_validate(sensor.parameters)
    for obj in run.objects:
        if obj.model not in registry.objects:
            raise ValueError(f"Unknown object model: {obj.model}")
