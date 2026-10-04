"""Native illustration-only sphere and z=0 ground protection previews."""

from pydrake.geometry import Box, GeometryInstance, MakePhongIllustrationProperties
from pydrake.geometry import Sphere as DrakeSphere
from pydrake.math import RigidTransform

from robo_arch.core.controllers.cbf.definition import Sphere
from robo_arch.core.worlds.drake.scene import DrakeScene


def add_sphere_illustrations(
    scene: DrakeScene, spheres: tuple[Sphere, ...], protected: tuple[str, ...]
) -> None:
    """Attach transparent illustration-only geometry; no contact or RGB-D changes."""
    source = scene.plant.get_source_id()
    for sphere in spheres:
        color = (
            [0.1, 0.6, 1.0, 0.24]
            if any(sphere.name.startswith(name + "/") for name in protected)
            else [0.9, 0.55, 0.15, 0.12]
        )
        geometry = GeometryInstance(
            RigidTransform(sphere.center),
            DrakeSphere(sphere.radius),
            "protection/" + sphere.name,
        )
        properties = MakePhongIllustrationProperties(color)
        properties.AddProperty("meshcat", "accepting", "protections")
        geometry.set_illustration_properties(properties)
        if sphere.frame == "world":
            scene.scene_graph.RegisterAnchoredGeometry(source, geometry)
        else:
            instance, frame_name = sphere.frame.rsplit("/", 1)
            model = scene.plant.GetModelInstanceByName(instance)
            frame = scene.plant.GetFrameByName(frame_name, model)
            geometry.set_pose(
                frame.GetFixedPoseInBodyFrame() @ RigidTransform(sphere.center)
            )
            scene.scene_graph.RegisterGeometry(
                source,
                scene.plant.GetBodyFrameIdOrThrow(frame.body().index()),
                geometry,
            )


def add_ground_illustrations(scene: DrakeScene, margin: float) -> None:
    """Preview the infinite sphere-bottom limit with a finite 10 m cyan grid.

    Grid and border upper faces are exactly at z=margin. The translucent fill
    sits 0.2 mm below them to avoid coplanar surfaces. All geometry is anchored
    illustration in the protections layer; the physical floor remains at z=0.
    """
    source = scene.plant.get_source_id()

    def add(
        name: str,
        size: tuple[float, float, float],
        center: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> None:
        geometry = GeometryInstance(
            RigidTransform(center), Box(*size), "protection/ground/" + name
        )
        properties = MakePhongIllustrationProperties(color)
        properties.AddProperty("meshcat", "accepting", "protections")
        geometry.set_illustration_properties(properties)
        scene.scene_graph.RegisterAnchoredGeometry(source, geometry)

    add(
        "fill",
        (10.0, 10.0, 0.001),
        (0.0, 0.0, margin - 0.0007),
        (0.05, 0.75, 0.95, 0.10),
    )
    for index in range(1, 20):
        offset = -5.0 + 0.5 * index
        add(
            f"grid/x_{index}",
            (10.0, 0.003, 0.0005),
            (0.0, offset, margin - 0.00025),
            (0.05, 0.75, 0.95, 0.65),
        )
        add(
            f"grid/y_{index}",
            (0.003, 10.0, 0.0005),
            (offset, 0.0, margin - 0.00025),
            (0.05, 0.75, 0.95, 0.65),
        )
    for index, offset in enumerate((-4.996, 4.996)):
        add(
            f"border/x_{index}",
            (10.0, 0.008, 0.001),
            (0.0, offset, margin - 0.0005),
            (0.05, 0.8, 1.0, 0.85),
        )
        add(
            f"border/y_{index}",
            (0.008, 10.0, 0.001),
            (offset, 0.0, margin - 0.0005),
            (0.05, 0.8, 1.0, 0.85),
        )
