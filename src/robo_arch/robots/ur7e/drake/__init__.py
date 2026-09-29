"""Drake model loading for the UR7e."""

from importlib.resources import as_file, files

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


def add_to_plant(plant: MultibodyPlant, *, name: str) -> ModelInstanceIndex:
    """Add the packaged arm, leaving base placement and finalization to the caller."""
    resource = files("robo_arch.robots.ur7e").joinpath("assets/model.urdf")
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    with as_file(resource) as path:
        (instance,) = parser.AddModels(str(path))
    plant.RenameModelInstance(instance, name)
    return instance
