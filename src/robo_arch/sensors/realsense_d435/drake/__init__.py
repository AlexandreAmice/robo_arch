"""Drake rendering adapter for the ideal pinhole camera."""

from pydrake.geometry import (
    ClippingRange,
    ColorRenderCamera,
    DepthRange,
    DepthRenderCamera,
    FrameId,
    MakeRenderEngineVtk,
    RenderCameraCore,
    RenderEngineVtkParams,
    SceneGraph,
)
from pydrake.math import RigidTransform
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.sensors import CameraInfo, RgbdSensor

from robo_arch.sensors.realsense_d435.definition import CameraParameters

RENDERER_NAME = "realsense_d435"


def add_to_builder(
    builder: DiagramBuilder,
    scene_graph: SceneGraph,
    *,
    parent_frame_id: FrameId,
    X_PB: RigidTransform,
    parameters: CameraParameters | None = None,
) -> RgbdSensor:
    """Attach a camera to a SceneGraph frame before building the diagram.

    ``X_PB`` locates the camera optical frame B relative to parent P. Images
    are evaluated on demand at the simulation context time; there is no
    sampling delay, noise or lens distortion. Depth is optical z in meters.
    The returned Drake system exposes RGBA8 and depth32F image output ports.
    VTK uses a headless EGL context on Linux by default; a working OpenGL
    implementation is required, whether hardware or software rendered.
    """
    parameters = parameters or CameraParameters()
    if not scene_graph.HasRenderer(RENDERER_NAME):
        scene_graph.AddRenderer(
            RENDERER_NAME, MakeRenderEngineVtk(RenderEngineVtkParams())
        )
    core = RenderCameraCore(
        RENDERER_NAME,
        CameraInfo(
            parameters.width, parameters.height, parameters.vertical_fov_radians
        ),
        ClippingRange(parameters.near_m, parameters.far_m),
        RigidTransform(),
    )
    sensor = builder.AddSystem(
        RgbdSensor(
            parent_frame_id,
            X_PB,
            ColorRenderCamera(
                RenderCameraCore(
                    RENDERER_NAME,
                    CameraInfo(
                        parameters.width,
                        parameters.height,
                        parameters.vertical_fov_radians,
                    ),
                    ClippingRange(parameters.near_m, parameters.far_m),
                    RigidTransform([-0.015, 0, 0]),
                ),
                show_window=False,
            ),
            DepthRenderCamera(core, DepthRange(parameters.near_m, parameters.far_m)),
        )
    )
    builder.Connect(
        scene_graph.get_query_output_port(), sensor.query_object_input_port()
    )
    return sensor
