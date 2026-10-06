"""Strict conversion of the supported fixed SDF box subset to USD colliders."""

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from importlib.resources import files

from robo_arch.core.config.declarations import ObjectInstance, SceneConfiguration
from robo_arch.core.worlds.devices import DeviceDefinitions


@dataclass(frozen=True)
class BoxFixture:
    instance: ObjectInstance
    size: tuple[float, ...]
    rgba: tuple[float, ...]
    mass: float | None
    inertia: tuple[float, ...] | None
    friction: tuple[float, float]


# Every accepted element/attribute is consumed below, except inertial data:
# fixtures are fixed to world, so their mass does not participate in dynamics.
_CHILDREN = {
    "sdf": ("model",),
    "model": ("link",),
    "link": ("inertial", "visual", "collision"),
    "inertial": ("mass", "inertia"),
    "inertia": ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
    "visual": ("geometry", "material"),
    "collision": ("geometry", "surface"),
    "surface": ("friction",),
    "friction": ("ode",),
    "ode": ("mu", "mu2"),
    "geometry": ("box",),
    "box": ("size",),
    "material": ("diffuse",),
}
_LEAVES = {
    "mass",
    "ixx",
    "iyy",
    "izz",
    "ixy",
    "ixz",
    "iyz",
    "size",
    "diffuse",
    "mu",
    "mu2",
}


def _validate(element: ET.Element, owner: str) -> None:
    attributes = (
        {"version"}
        if element.tag == "sdf"
        else {"name"}
        if element.tag in {"model", "link", "visual", "collision"}
        else set()
    )
    if set(element.attrib) - attributes:
        raise ValueError(f"Isaac fixture {owner}: unsupported {element.tag} attributes")
    if element.tag in _LEAVES:
        if len(element):
            raise ValueError(f"Isaac fixture {owner}: unexpected nested {element.tag}")
        return
    if element.tag not in _CHILDREN or (element.text or "").strip():
        raise ValueError(
            f"Isaac fixture {owner}: unsupported SDF element {element.tag}"
        )
    seen = set()
    for child in element:
        if child.tag not in _CHILDREN[element.tag] or child.tag in seen:
            raise ValueError(
                f"Isaac fixture {owner}: unsupported or repeated {element.tag}/{child.tag}"
            )
        if (child.tail or "").strip():
            raise ValueError(f"Isaac fixture {owner}: unexpected text in {element.tag}")
        seen.add(child.tag)
        _validate(child, owner)


def _numbers(root: ET.Element, path: str, count: int, owner: str) -> tuple[float, ...]:
    value = tuple(float(v) for v in root.findtext(path, "").split())
    if len(value) != count or not all(math.isfinite(v) for v in value):
        raise ValueError(
            f"Isaac fixture {owner}: expected {count} finite values at {path}"
        )
    return value


def load_boxes(
    scene: SceneConfiguration, definitions: DeviceDefinitions
) -> tuple[BoxFixture, ...]:
    """Validate without an SDK; reject any asset meaning we cannot translate.

    Supported assets are SDF 1.7, one link at its declared base frame, one box
    visual and one identical box collision, optional diffuse RGBA and inertia.
    Offsets, additional geometry, contact properties and movable bodies require
    a different importer; they are never silently discarded here.
    """
    result = []
    for instance in scene.objects:
        definition = definitions.objects[instance.model]
        root = ET.fromstring(
            files(definition.package).joinpath(definition.resource).read_text()
        )
        owner = instance.name
        if root.tag != "sdf" or root.get("version") != "1.7":
            raise ValueError(f"Isaac fixture {owner}: expected SDF 1.7")
        _validate(root, owner)
        link = root.find("model/link")
        if link is None or link.get("name") != definition.base_frame:
            raise ValueError(f"Isaac fixture {owner}: missing declared base frame")
        size = _numbers(link, "visual/geometry/box/size", 3, owner)
        collision = _numbers(link, "collision/geometry/box/size", 3, owner)
        if size != collision or not all(v > 0 for v in size):
            raise ValueError(
                f"Isaac fixture {owner}: expected matching positive box sizes"
            )
        mass, inertia = None, None
        inertial = link.find("inertial")
        if inertial is not None:
            if _numbers(inertial, "mass", 1, owner)[0] <= 0:
                raise ValueError(f"Isaac fixture {owner}: mass must be positive")
            for axis in ("ixx", "iyy", "izz"):
                if _numbers(inertial, "inertia/" + axis, 1, owner)[0] <= 0:
                    raise ValueError(f"Isaac fixture {owner}: inertia must be positive")
            for axis in ("ixy", "ixz", "iyz"):
                if inertial.find("inertia/" + axis) is not None:
                    _numbers(inertial, "inertia/" + axis, 1, owner)
            mass = _numbers(inertial, "mass", 1, owner)[0]
            inertia = tuple(
                float(inertial.findtext("inertia/" + key, "0"))
                for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
            )
            import numpy as np

            xx, yy, zz, xy, xz, yz = inertia
            eigenvalues = np.linalg.eigvalsh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
            if eigenvalues[0] <= 0 or eigenvalues[-1] > eigenvalues[:2].sum() + 1e-12:
                raise ValueError(f"Isaac object {owner}: invalid rigid-body inertia")
        if instance.motion == "free" and mass is None:
            raise ValueError(f"Isaac object {owner}: free body requires inertia")
        friction = (1.0, 1.0)
        if link.find("collision/surface") is not None:
            friction = tuple(
                _numbers(link, "collision/surface/friction/ode/" + key, 1, owner)[0]
                for key in ("mu", "mu2")
            )
            if not 0 <= friction[1] <= friction[0]:
                raise ValueError(f"Isaac object {owner}: invalid friction")
        rgba = (
            _numbers(link, "visual/material/diffuse", 4, owner)
            if link.find("visual/material") is not None
            else (0.5, 0.5, 0.5, 1.0)
        )
        if not all(0 <= v <= 1 for v in rgba):
            raise ValueError(f"Isaac fixture {owner}: diffuse RGBA must be in [0, 1]")
        result.append(BoxFixture(instance, size, rgba, mass, inertia, friction))
    return tuple(result)


def add_objects(
    stage, scene: SceneConfiguration, definitions: DeviceDefinitions, *, root: str = ""
) -> dict[str, str]:
    """Create fixed and free boxes; return native free-body paths by instance."""
    import numpy as np
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    from robo_arch.core.worlds.assembly import pose_transform

    dynamic = {}
    for fixture in load_boxes(scene, definitions):
        path = root + "/objects/" + fixture.instance.name
        body = UsdGeom.Xform.Define(stage, path)
        transform = pose_transform(fixture.instance.pose).GetAsMatrix4()
        body.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))
        box = UsdGeom.Cube.Define(stage, path + "/geometry")
        box.CreateSizeAttr(1.0)
        box.AddScaleOp().Set(Gf.Vec3f(*fixture.size))
        box.CreateDisplayColorAttr([Gf.Vec3f(*fixture.rgba[:3])])
        box.CreateDisplayOpacityAttr([fixture.rgba[3]])
        UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        material = UsdShade.Material.Define(stage, path + "/material")
        physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        physics.CreateStaticFrictionAttr(fixture.friction[0])
        physics.CreateDynamicFrictionAttr(fixture.friction[1])
        physics.CreateRestitutionAttr(0.0)
        UsdShade.MaterialBindingAPI.Apply(box.GetPrim()).Bind(
            material, materialPurpose="physics"
        )
        if fixture.instance.motion == "free":
            UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            mass = UsdPhysics.MassAPI.Apply(body.GetPrim())
            mass.CreateMassAttr(fixture.mass)
            xx, yy, zz, xy, xz, yz = fixture.inertia
            moments, axes = np.linalg.eigh([[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]])
            if np.linalg.det(axes) < 0:
                axes[:, 0] *= -1
            mass.CreateDiagonalInertiaAttr(Gf.Vec3f(*moments))
            rotation = Gf.Matrix3d(axes.T.tolist()).ExtractRotation()
            mass.CreatePrincipalAxesAttr(Gf.Quatf(rotation.GetQuat()))
            mass.CreateCenterOfMassAttr(Gf.Vec3f(0, 0, 0))
            dynamic[fixture.instance.name] = path
    return dynamic
