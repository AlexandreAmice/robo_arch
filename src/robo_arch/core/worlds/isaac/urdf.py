"""Anchor a converted fixed-base URDF articulation and remove actuator drives."""

import subprocess
import sys
from pathlib import Path

import numpy as np


def add_to_stage(
    stage, *, name: str, X_WB: np.ndarray, directory: Path, urdf: Path
) -> str:
    """Return the fixed articulation root path; no position drives are installed.

    Conversion runs separately because the converter and Kit ship incompatible
    USD libraries. The converted asset is temporary and uses the same URDF as
    Drake; no robot geometry or dynamics parameters are duplicated here.
    """
    from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

    subprocess.run(
        [sys.executable, "-m", __package__ + ".convert", str(urdf), str(directory)],
        check=True,
    )
    path = "/" + name
    root = stage.DefinePrim(path, "Xform")
    root.GetReferences().AddReference(str(directory / (urdf.stem + ".usdc")))
    matrix = Gf.Matrix4d(X_WB.T.tolist())
    UsdGeom.Xformable(root).AddTransformOp().Set(matrix)
    articulation_bodies = [
        p
        for p in stage.Traverse()
        if p.GetPath().HasPrefix(root.GetPath())
        and p.HasAPI(UsdPhysics.ArticulationRootAPI)
    ]
    if len(articulation_bodies) != 1:
        raise ValueError("URDF conversion must contain exactly one articulation")
    body = articulation_bodies[0]
    fixed_joints = [
        UsdPhysics.FixedJoint(p)
        for p in stage.Traverse()
        if p.IsA(UsdPhysics.FixedJoint)
        and UsdPhysics.Joint(p).GetBody1Rel().GetTargets() == [body.GetPath()]
    ]
    if len(fixed_joints) != 1:
        raise ValueError("URDF conversion must contain one fixed base joint")
    joint = fixed_joints[0]
    # World is the empty body0 relationship. Its local frame follows base pose.
    joint.GetBody0Rel().ClearTargets(True)
    local = Gf.Matrix4d(1)
    local.SetRotate(Gf.Quatd(joint.GetLocalRot0Attr().Get()))
    local.SetTranslateOnly(Gf.Vec3d(joint.GetLocalPos0Attr().Get()))
    world = local * matrix
    joint.GetLocalPos0Attr().Set(Gf.Vec3f(world.ExtractTranslation()))
    joint.GetLocalRot0Attr().Set(Gf.Quatf(world.ExtractRotationQuat()))
    body.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    UsdPhysics.ArticulationRootAPI.Apply(joint.GetPrim())
    for prim in stage.Traverse():
        if prim.GetPath().HasPrefix(root.GetPath()):
            for drive in ("angular", "linear"):
                if prim.HasAPI(UsdPhysics.DriveAPI, drive):
                    prim.RemoveAPI(UsdPhysics.DriveAPI, drive)
    PhysxSchema.PhysxArticulationAPI.Apply(
        joint.GetPrim()
    ).CreateEnabledSelfCollisionsAttr(True)
    # Native articulation topology excludes parent-child collision pairs only.
    return str(joint.GetPath())
