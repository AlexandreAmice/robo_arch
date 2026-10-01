"""The renderer reports a known box surface in optical depth coordinates."""

import pytest

pytest.importorskip("pydrake")

import numpy as np
from pydrake.geometry import Box
from pydrake.math import RigidTransform
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

from robo_arch.sensors.realsense_d435.definition import CameraParameters
from robo_arch.sensors.realsense_d435.drake import add_to_builder


def test_rgbd_renders_box_at_known_depth():
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.001)
    plant.RegisterVisualGeometry(
        plant.world_body(),
        RigidTransform([0.0, 0.0, 1.0]),
        Box(0.4, 0.4, 0.2),
        "box",
        [1.0, 0.0, 0.0, 1.0],
    )
    plant.Finalize()
    camera = add_to_builder(
        builder,
        scene_graph,
        parent_frame_id=scene_graph.world_frame_id(),
        X_PB=RigidTransform(),
    )
    diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    camera_context = camera.GetMyContextFromRoot(context)
    depth = camera.depth_image_32F_output_port().Eval(camera_context).data
    rgba = camera.color_image_output_port().Eval(camera_context).data
    assert depth.shape == (48, 64, 1)
    assert depth[24, 32, 0] == pytest.approx(0.9, abs=1e-5)
    assert np.isinf(depth[0, 0, 0])
    assert rgba.shape == (48, 64, 4)
    assert rgba[24, 32, 0] > rgba[24, 32, 1]
    assert rgba[24, 32, 3] == 255


def test_invalid_depth_range_is_rejected():
    with pytest.raises(ValueError, match="near_m must be less than far_m"):
        CameraParameters(near_m=2.0, far_m=1.0)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
