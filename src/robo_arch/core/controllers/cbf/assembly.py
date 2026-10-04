"""Resolve protection profiles and explicit exclusions without a physics SDK."""

from dataclasses import dataclass, replace

import numpy as np

from robo_arch.core.config.declarations import SceneConfiguration
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.cbf.definition import (
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)
from robo_arch.core.controllers.cbf.geometry import _rotation, load_sphere_profile


@dataclass(frozen=True)
class ProtectionGeometry:
    """Resolved coverings and enabled constraints in deterministic profile order."""

    spheres: tuple[Sphere, ...]
    pairs: tuple[SpherePair, ...]
    planes: tuple[Plane, ...]
    plane_pairs: tuple[SpherePlanePair, ...]


def resolve_geometry(
    scene: SceneConfiguration,
    parameters: ProtectionParameters,
    *,
    ground: bool = False,
) -> ProtectionGeometry:
    """Build instance-qualified protection geometry for a physical scene.

    Single-link fixed object centers are transformed into world coordinates.
    Object profiles with additional link/frame transforms fail explicitly.
    Device spheres
    retain their model-local frames for the selected runtime to resolve. Pairs
    connect different profile instances when at least one is protected, except
    explicitly excluded frame pairs. Ground adds the fixed world z=0 halfspace
    for each protected sphere. Neither declarations nor resolution require an SDK.
    """
    objects = {obj.name: obj for obj in scene.objects}
    grouped = {}
    for instance, profile in parameters.profiles.items():
        spheres = load_sphere_profile(
            profile, instance, fixed_object=instance in objects
        )
        if instance in objects:
            pose = objects[instance].pose
            rotation = _rotation(pose.rpy)
            translation = np.asarray(pose.translation)
            spheres = tuple(
                replace(
                    sphere,
                    frame="world",
                    center=tuple(
                        float(value)
                        for value in (
                            rotation @ np.asarray(sphere.center) + translation
                        )
                    ),
                )
                for sphere in spheres
            )
        grouped[instance] = spheres
    frames = {sphere.frame for values in grouped.values() for sphere in values}
    exclusions = {frozenset(pair) for pair in parameters.exclude_frames}
    if len(exclusions) != len(parameters.exclude_frames):
        raise ValueError("Duplicate mounting exclusion")
    if any(len(pair) != 2 or not pair <= frames for pair in exclusions):
        raise ValueError("Mount exclusions must name two distinct covered frames")
    pairs = []
    for first_name, first_spheres in grouped.items():
        for second_name, second_spheres in grouped.items():
            if first_name >= second_name:
                continue
            if not {first_name, second_name}.intersection(parameters.protected):
                continue
            for first in first_spheres:
                for second in second_spheres:
                    if frozenset((first.frame, second.frame)) not in exclusions:
                        pairs.append(
                            SpherePair(first.name, second.name, parameters.margin)
                        )
    spheres = tuple(sphere for values in grouped.values() for sphere in values)
    planes = (Plane("ground", (0.0, 0.0, 1.0), 0.0),) if ground else ()
    plane_pairs = tuple(
        SpherePlanePair(sphere.name, "ground", parameters.margin)
        for instance, instance_spheres in grouped.items()
        if ground and instance in parameters.protected
        for sphere in instance_spheres
    )
    return ProtectionGeometry(spheres, tuple(pairs), planes, plane_pairs)
