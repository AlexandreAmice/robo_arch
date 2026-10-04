"""Conservative coverage, asset transforms and strict profile loading."""

from itertools import product

import numpy as np
import pytest

from robo_arch.core.controllers.cbf import geometry


def test_box_cover_contains_interior_and_boundary_without_one_large_sphere():
    lower, upper = np.array([-0.2, -0.1, 0]), np.array([0.2, 0.1, 0.2])
    spheres = geometry.cover_box(lower, upper, 0.1)
    centers, radii = zip(*spheres, strict=True)
    points = np.array(
        list(
            product(*(np.linspace(a, b, 17) for a, b in zip(lower, upper, strict=True)))
        )
    )
    distances = np.linalg.norm(points[:, None] - np.asarray(centers), axis=2)
    assert np.all(np.min(distances - radii, axis=1) <= 1e-12)
    assert max(radii) <= np.sqrt(3) * 0.1 / 2 + 1e-12
    assert len(spheres) > 1


def test_asset_mesh_scale_and_collision_origin_are_included(tmp_path):
    (tmp_path / "shape.obj").write_text(
        "v 0 0 0\n  v\t2 0 0\nv\t0 1 0\n\tv 0 0 0.5\n"
        "f 1 2 3\nf 1 2 4\nf 1 3 4\nf 2 3 4\n"
    )
    asset = tmp_path / "model.urdf"
    asset.write_text("""<robot name="fixture"><link name="body"><collision>
        <origin xyz="1 2 3" rpy="0 0 1.5707963267948966"/>
        <geometry><mesh filename="shape.obj" scale="2 3 4"/></geometry>
        </collision></link></robot>""")
    ((frame, lower, upper),) = geometry.collision_boxes(asset)
    assert frame == "body"
    np.testing.assert_allclose(lower, [-2, 2, 3], atol=1e-12)
    np.testing.assert_allclose(upper, [1, 6, 5], atol=1e-12)


@pytest.mark.parametrize(
    "extra",
    [
        '<link name="other"/>',
        '<frame name="offset" link="body" xyz="2 0 0"/>',
        '<joint name="offset" type="fixed"/>',
    ],
)
def test_fixed_object_profiles_reject_unresolved_frames(tmp_path, extra):
    asset = tmp_path / "model.urdf"
    asset.write_text(f"""<robot name="fixture"><link name="body"><collision>
        <geometry><box size="1 2 3"/></geometry></collision></link>
        {extra}</robot>""")
    with pytest.raises(ValueError, match="Fixed-object.*single-link"):
        geometry.collision_boxes(asset, fixed_object=True)
    # Frame-attached robot/device coverings retain their local link coordinates.
    assert geometry.collision_boxes(asset)[0][0] == "body"


def test_fixed_object_profiles_reject_nested_sdf_models(tmp_path):
    asset = tmp_path / "model.sdf"
    asset.write_text("""<sdf version="1.7"><model name="fixture">
        <model name="nested"><link name="body"><collision name="box">
        <geometry><box><size>1 2 3</size></box></geometry>
        </collision></link></model></model></sdf>""")
    with pytest.raises(ValueError, match="nested models"):
        geometry.collision_boxes(asset, fixed_object=True)


@pytest.mark.parametrize(
    "attributes",
    ['relative_to="other"', 'degrees="true"', 'rotation_format="quat_xyzw"'],
)
def test_unsupported_sdf_pose_conventions_are_rejected(tmp_path, attributes):
    asset = tmp_path / "model.sdf"
    asset.write_text(f"""<sdf version="1.7"><model name="fixture"><link name="body">
        <collision name="box"><pose {attributes}>0 0 0 0 0 0</pose>
        <geometry><box><size>1 2 3</size></box></geometry>
        </collision></link></model></sdf>""")
    with pytest.raises(ValueError, match="pose attributes"):
        geometry.collision_boxes(asset)


@pytest.mark.parametrize("size", ["1 -2 3", "1 0 3", "1 nan 3", "1 2"])
def test_invalid_box_dimensions_fail(tmp_path, size):
    asset = tmp_path / "model.urdf"
    asset.write_text(f'''<robot name="fixture"><link name="body"><collision>
        <geometry><box size="{size}"/></geometry></collision></link></robot>''')
    with pytest.raises(ValueError, match="box size"):
        geometry.collision_boxes(asset)


@pytest.mark.parametrize("cell_size", [0, -0.1, np.nan, np.inf])
def test_invalid_subdivision_is_rejected(cell_size):
    with pytest.raises(ValueError, match="cell size"):
        geometry.cover_box([0, 0, 0], [1, 1, 1], cell_size)


def test_profile_rejects_duplicate_yaml_keys(tmp_path, monkeypatch):
    profile = tmp_path / "profile.yaml"
    profile.write_text(
        "asset: package://robo_arch/first.urdf\n"
        "asset: package://robo_arch/second.urdf\ncell_size: 0.1\n"
    )
    monkeypatch.setattr(geometry, "resolve_resource", lambda resource: profile)
    with pytest.raises(ValueError, match="Duplicate YAML key"):
        geometry.load_sphere_profile("package://robo_arch/profile.yaml", "camera")


def test_profile_rejects_working_directory_relative_asset():
    with pytest.raises(ValueError, match="package://"):
        geometry.SphereProfile(asset="model.urdf", cell_size=0.1)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
