"""Conservative sphere coverings of collision geometry, without simulator imports.

Every cell in a collision geometry's axis-aligned bounding box is enclosed by
its circumscribed sphere. This covers the entire box (and therefore the convex
mesh), including its interior; it is not a sampled surface approximation.
"""

from itertools import product
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from numpy.typing import ArrayLike
from pydantic import Field, field_validator

from robo_arch.core.config.loading import read_validated_yaml, resolve_resource
from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.resources import validate_package_reference
from robo_arch.core.controllers.cbf.definition import Sphere


class SphereProfile(Parameters):
    """Model-owned collision asset and maximum subdivision cell size in metres."""

    asset: str
    cell_size: float = Field(gt=0, allow_inf_nan=False)

    @field_validator("asset")
    @classmethod
    def package_asset(cls, value: str) -> str:
        return validate_package_reference(value)


def cover_box(
    lower: ArrayLike, upper: ArrayLike, cell_size: float
) -> tuple[tuple[np.ndarray, float], ...]:
    """Enclose each complete AABB cell; return owned centers and radii in metres."""
    lower, upper = np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)
    if (
        lower.shape != (3,)
        or upper.shape != (3,)
        or not np.isfinite([lower, upper]).all()
        or np.any(upper < lower)
        or not np.isfinite(cell_size)
        or cell_size <= 0
    ):
        raise ValueError("Sphere coverage requires finite ordered bounds and cell size")
    counts = np.maximum(1, np.ceil((upper - lower) / cell_size).astype(int))
    widths = (upper - lower) / counts
    radius = float(np.linalg.norm(widths) / 2)
    if radius == 0:
        raise ValueError("Collision geometry cannot have zero extent")
    return tuple(
        (lower + (np.asarray(index) + 0.5) * widths, radius)
        for index in product(*(range(int(count)) for count in counts))
    )


def _rotation(rpy: ArrayLike) -> np.ndarray:
    r, p, y = np.asarray(rpy, dtype=float)
    cr, cp, cy = np.cos([r, p, y])
    sr, sp, sy = np.sin([r, p, y])
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _numbers(text: str) -> np.ndarray:
    return np.asarray([float(value) for value in text.split()])


def collision_boxes(
    asset: Path, *, fixed_object: bool = False
) -> tuple[tuple[str, np.ndarray, np.ndarray], ...]:
    """Read bounds in each link frame from supported URDF/SDF collision assets.

    Supports OBJ meshes and boxes; unsupported geometry fails explicitly. Mesh
    bounds are transformed before subdivision, preserving complete coverage.
    SDF link/model poses must be identity; collision poses may be nonidentity.
    Fixed objects must have exactly one link and no joints or extra frames;
    otherwise applying a base pose to link-local bounds would lose transforms.
    """
    tree = ElementTree.parse(asset)
    sdf = tree.getroot().tag == "sdf"
    if tree.getroot().tag not in {"robot", "sdf"}:
        raise ValueError(f"Unsupported collision asset {asset}")
    if fixed_object and (
        len(tree.findall(".//link")) != 1
        or tree.findall(".//joint")
        or tree.findall(".//frame")
        or (sdf and len(tree.findall(".//model")) != 1)
    ):
        raise ValueError(
            "Fixed-object sphere profiles require a single-link asset without "
            "joints, extra frames or nested models; base-to-link transforms "
            f"are not supported: {asset}"
        )
    if sdf:
        for element in tree.findall(".//model") + tree.findall(".//link"):
            if element.find("pose") is not None:
                raise ValueError(
                    "Sphere coverage does not support SDF model/link poses"
                )
    boxes = []
    for link in tree.findall(".//link"):
        for collision in link.findall("collision"):
            geometry = collision.find("geometry")
            if geometry is None or len(geometry) != 1:
                raise ValueError(f"Expected one collision geometry in {asset}")
            mesh, box = geometry.find("mesh"), geometry.find("box")
            if mesh is not None:
                filename = mesh.findtext("uri") if sdf else mesh.get("filename")
                path = (
                    resolve_resource(filename)
                    if filename.startswith("package://")
                    else asset.parent / filename
                )
                if path.suffix != ".obj":
                    raise ValueError(f"Sphere coverage requires an OBJ mesh: {path}")
                records = [
                    tokens
                    for entry in path.read_text().splitlines()
                    if (tokens := entry.split()) and tokens[0] == "v"
                ]
                if not records or any(len(tokens) < 4 for tokens in records):
                    raise ValueError(
                        f"OBJ mesh needs complete vertex coordinates: {path}"
                    )
                vertices = np.array(
                    [[float(value) for value in tokens[1:4]] for tokens in records]
                )
                scale = (
                    mesh.findtext("scale", "1 1 1")
                    if sdf
                    else mesh.get("scale", "1 1 1")
                )
                vertices *= _numbers(scale)
            elif box is not None:
                size = _numbers(box.findtext("size") if sdf else box.get("size"))
                if (
                    size.shape != (3,)
                    or not np.isfinite(size).all()
                    or np.any(size <= 0)
                ):
                    raise ValueError(f"Invalid collision box size in {asset}")
                vertices = np.asarray(
                    list(product(*zip(-size / 2, size / 2, strict=True)))
                )
            else:
                raise ValueError(
                    f"Unsupported collision shape in {asset}:{link.get('name')}"
                )
            if sdf:
                pose_element = collision.find("pose")
                if pose_element is not None and pose_element.attrib:
                    raise ValueError(
                        "Sphere coverage does not support SDF pose attributes"
                    )
                pose = _numbers(collision.findtext("pose", "0 0 0 0 0 0"))
                if pose.shape != (6,):
                    raise ValueError("SDF collision pose needs six values")
                translation, rpy = pose[:3], pose[3:]
            else:
                origin = collision.find("origin")
                translation = (
                    _numbers(origin.get("xyz", "0 0 0"))
                    if origin is not None
                    else np.zeros(3)
                )
                rpy = (
                    _numbers(origin.get("rpy", "0 0 0"))
                    if origin is not None
                    else np.zeros(3)
                )
            if vertices.ndim != 2 or vertices.shape[1:] != (3,) or not len(vertices):
                raise ValueError(
                    f"Collision mesh needs three-dimensional vertices: {asset}"
                )
            if (
                not np.isfinite(vertices).all()
                or translation.shape != (3,)
                or rpy.shape != (3,)
                or not np.isfinite([translation, rpy]).all()
            ):
                raise ValueError(f"Nonfinite or invalid collision transform in {asset}")
            points = vertices @ _rotation(rpy).T + translation
            boxes.append((link.get("name"), points.min(axis=0), points.max(axis=0)))
    if not boxes:
        raise ValueError(f"No collision geometry in {asset}")
    return tuple(boxes)


def load_sphere_profile(
    resource: str, instance: str, *, fixed_object: bool = False
) -> tuple[Sphere, ...]:
    """Instantiate a model-owned profile with explicit instance/frame identity."""
    profile = read_validated_yaml(resolve_resource(resource), SphereProfile)
    asset = resolve_resource(profile.asset)
    spheres = []
    for collision_index, (body, lower, upper) in enumerate(
        collision_boxes(asset, fixed_object=fixed_object)
    ):
        for cell_index, (center, radius) in enumerate(
            cover_box(lower, upper, profile.cell_size)
        ):
            spheres.append(
                Sphere(
                    name=f"{instance}/{body}/{collision_index}/{cell_index}",
                    frame=f"{instance}/{body}",
                    center=tuple(float(value) for value in center),
                    radius=radius,
                )
            )
    return tuple(spheres)
