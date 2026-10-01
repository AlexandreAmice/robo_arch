"""Seven distinct iiwa actuators and positive nominal dynamics."""

import numpy as np
import pytest

pytest.importorskip("pydrake")
from pydrake.multibody.plant import MultibodyPlant

from robo_arch.core.config.loading import load_robot
from robo_arch.core.worlds.drake.models import add_robot


def test_seven_joint_model():
    definition = load_robot("package://robo_arch/robots/iiwa7/robot.yaml")
    model = MultibodyPlant(0.001)
    instance = add_robot(model, definition, name="arm")
    model.WeldFrames(
        model.world_frame(), model.GetFrameByName(definition.base_frame, instance)
    )
    model.Finalize()
    assert (
        tuple(
            model.get_joint_actuator(i).joint().name()
            for i in model.GetJointActuatorIndices()
        )
        == definition.joints
    )
    assert model.num_positions() == model.num_actuators() == 7
    context = model.CreateDefaultContext()
    model.SetPositions(context, definition.default_positions)
    assert np.linalg.eigvalsh(model.CalcMassMatrix(context)).min() > 0


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
