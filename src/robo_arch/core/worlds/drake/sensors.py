"""Physical sensor mounting before plant finalization, independent of observations."""

from importlib.resources import as_file, files

from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import Frame, ModelInstanceIndex

from robo_arch.core.config.declarations import SensorDefinition, SensorInstance


def add_sensor_body(
    plant: MultibodyPlant,
    sensor: SensorInstance,
    definition: SensorDefinition,
    parent: Frame,
    X_PM: RigidTransform,
) -> ModelInstanceIndex:
    """Weld the device-owned canonical asset; return its separate model instance."""
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    asset = files(f"robo_arch.sensors.{sensor.model}").joinpath(definition.resource)
    with as_file(asset) as path:
        (instance,) = parser.AddModels(str(path))
    plant.RenameModelInstance(instance, sensor.name)
    plant.WeldFrames(
        parent, plant.GetFrameByName(definition.base_frame, instance), X_PM
    )
    return instance
