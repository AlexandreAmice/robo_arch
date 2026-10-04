"""Validate and index protection geometry once for every execution backend."""

from dataclasses import dataclass

import numpy as np

from robo_arch.core.controllers.cbf.definition import (
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)


@dataclass(frozen=True)
class ConstraintLayout[Array]:
    """Model-independent pair ordering and owned CPU constants in SI units."""

    pair_names: tuple[str, ...]
    first: Array
    second: Array
    separation: Array
    local_centers: Array
    plane_spheres: Array
    plane_normals: Array
    plane_offsets: Array


def compile_geometry(
    spheres: tuple[Sphere, ...],
    pairs: tuple[SpherePair, ...],
    planes: tuple[Plane, ...],
    plane_pairs: tuple[SpherePlanePair, ...],
) -> ConstraintLayout[np.ndarray]:
    """Check references and preserve sphere-pair then sphere-plane row ordering."""
    if not spheres or not (pairs or plane_pairs):
        raise ValueError("CBF requires spheres and enabled pairs")
    names = [s.name for s in spheres]
    if len(set(names)) != len(names):
        raise ValueError("Sphere names must be unique")
    plane_names = [plane.name for plane in planes]
    if len(set(plane_names)) != len(plane_names):
        raise ValueError("Plane names must be unique")
    if set(names).intersection(plane_names):
        raise ValueError("Sphere and plane names must not overlap")
    keys = [frozenset((p.first, p.second)) for p in pairs]
    if len(set(keys)) != len(keys):
        raise ValueError("Sphere pairs must be unique, including reverse pairs")
    if any(not key.issubset(names) for key in keys):
        raise ValueError("Sphere pair references an unknown sphere")
    plane_keys = [(pair.sphere, pair.plane) for pair in plane_pairs]
    if len(set(plane_keys)) != len(plane_keys):
        raise ValueError("Sphere-plane pairs must be unique")
    if any(pair.sphere not in names for pair in plane_pairs):
        raise ValueError("Sphere-plane pair references an unknown sphere")
    if any(pair.plane not in plane_names for pair in plane_pairs):
        raise ValueError("Sphere-plane pair references an unknown plane")
    pair_names = tuple(f"{p.first}|{p.second}" for p in pairs) + tuple(
        f"{p.sphere}|{p.plane}" for p in plane_pairs
    )
    if len(set(pair_names)) != len(pair_names):
        raise ValueError("Protection pair display names must be unique")
    sphere_indices = {sphere.name: i for i, sphere in enumerate(spheres)}
    first = np.array([sphere_indices[p.first] for p in pairs], dtype=int)
    second = np.array([sphere_indices[p.second] for p in pairs], dtype=int)
    plane_by_name = {plane.name: plane for plane in planes}
    plane_spheres = np.array(
        [sphere_indices[pair.sphere] for pair in plane_pairs], dtype=int
    )
    return ConstraintLayout(
        pair_names=pair_names,
        first=first,
        second=second,
        separation=np.array(
            [
                spheres[i].radius + spheres[j].radius + pair.margin
                for i, j, pair in zip(first, second, pairs, strict=True)
            ]
        ),
        local_centers=np.array([s.center for s in spheres], dtype=float),
        plane_spheres=plane_spheres,
        plane_normals=np.array(
            [plane_by_name[pair.plane].normal for pair in plane_pairs], dtype=float
        ).reshape(-1, 3),
        plane_offsets=np.array(
            [
                plane_by_name[pair.plane].offset + spheres[index].radius + pair.margin
                for pair, index in zip(plane_pairs, plane_spheres, strict=True)
            ]
        ),
    )
