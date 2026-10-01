"""Ideal Mini45 wrench from the load-path joint, including tool-side gravity."""

import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import BasicVector, DiagramBuilder, LeafSystem


class WrenchSensor(LeafSystem):
    """Parent-on-child wrench, at and expressed in sensing frame; [F, T]."""

    def __init__(self, plant: MultibodyPlant, instance: ModelInstanceIndex):
        super().__init__()
        self._index = int(plant.GetJointByName("sensing_joint", instance).index())
        port = plant.get_reaction_forces_output_port()
        self._forces = self.DeclareAbstractInputPort("joint_reactions", port.Allocate())
        self.DeclareVectorOutputPort("wrench", BasicVector(6), self._output)

    def _output(self, context, output):
        force = self._forces.Eval(context)[self._index]
        output.SetFromVector(np.r_[force.translational(), force.rotational()])


def add_to_builder(builder: DiagramBuilder, plant: MultibodyPlant, *, instance):
    sensor = builder.AddSystem(WrenchSensor(plant, instance))
    builder.Connect(plant.get_reaction_forces_output_port(), sensor.get_input_port())
    return sensor
