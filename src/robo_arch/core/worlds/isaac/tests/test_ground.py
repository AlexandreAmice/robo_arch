"""Check ground declaration and native USD authoring without starting Kit."""

import pytest

from robo_arch.core.worlds.isaac import scene
from robo_arch.core.worlds.isaac.config import IsaacWorld


def test_ground_setting_defaults_on_and_round_trips():
    assert IsaacWorld().ground is True
    config = IsaacWorld(ground=False)
    assert IsaacWorld.model_validate_json(config.model_dump_json()).ground is False


@pytest.mark.parametrize("enabled", [True, False])
def test_scene_ground_authors_collision_visual_and_material(enabled):
    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    UsdPhysics = pytest.importorskip("pxr.UsdPhysics")
    UsdShade = pytest.importorskip("pxr.UsdShade")
    stage = Usd.Stage.CreateInMemory()
    scene.add_ground(stage, enabled=enabled)
    ground = stage.GetPrimAtPath("/_world/ground")
    assert bool(ground) == enabled
    if not enabled:
        return
    plane = UsdGeom.Plane.Get(stage, "/_world/ground/collision")
    assert plane and plane.GetAxisAttr().Get() == "Z"
    assert plane.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    assert not plane.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI)
    assert plane.ComputeLocalToWorldTransform(0).ExtractTranslation()[2] == 0
    material, _ = UsdShade.MaterialBindingAPI(plane.GetPrim()).ComputeBoundMaterial(
        materialPurpose="physics"
    )
    physics = UsdPhysics.MaterialAPI(material.GetPrim())
    assert physics.GetStaticFrictionAttr().Get() == pytest.approx(0.8)
    assert physics.GetDynamicFrictionAttr().Get() == pytest.approx(0.6)
    assert physics.GetRestitutionAttr().Get() == 0
    visual = UsdGeom.Mesh.Get(stage, "/_world/ground/visual")
    assert visual and not visual.GetPrim().HasAPI(UsdPhysics.CollisionAPI)
    points = list(visual.GetPointsAttr().Get())
    assert all(point[2] == 0 for point in points)
    assert min(point[0] for point in points) == -5
    assert max(point[0] for point in points) == 5
    assert min(point[1] for point in points) == -5
    assert max(point[1] for point in points) == 5
    assert visual.GetFaceVertexIndicesAttr().Get() == [0, 1, 2, 3]
    assert tuple(visual.GetDisplayColorAttr().Get()[0]) == pytest.approx(
        (0.55, 0.57, 0.60)
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
