"""Load declared robot assets into independent Drake model instances."""

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex

from robo_arch.core.config.declarations import RobotDefinition
from robo_arch.core.config.loading import resolve_resource


def add_robot(
    plant: MultibodyPlant, definition: RobotDefinition, *, name: str
) -> ModelInstanceIndex:
    """Parse the asset; leave base placement and finalization to the caller."""
    path = resolve_resource(definition.asset)
    if path.suffix != ".urdf":
        raise ValueError(f"Unsupported Drake robot asset: {definition.asset}")
    parser = Parser(plant)
    parser.SetAutoRenaming(True)
    (instance,) = parser.AddModels(str(path))
    plant.RenameModelInstance(instance, name)
    # Per-instance actuator indices are populated only after Finalize().
    joints = tuple(
        plant.get_joint_actuator(index).joint().name()
        for index in plant.GetJointActuatorIndices()
        if plant.get_joint_actuator(index).model_instance() == instance
    )
    if joints != definition.joints:
        raise ValueError(f"Robot {name} actuator order {joints} != {definition.joints}")
    plant.GetFrameByName(definition.base_frame, instance)
    return instance
