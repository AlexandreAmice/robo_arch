"""Load independently named iiwa models from the packaged canonical URDF."""

from importlib.resources import as_file, files

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


def add_to_plant(plant: MultibodyPlant, *, name: str) -> ModelInstanceIndex:
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    with as_file(files("robo_arch.robots.iiwa7").joinpath("assets/model.urdf")) as path:
        (instance,) = parser.AddModels(str(path))
    plant.RenameModelInstance(instance, name)
    return instance
