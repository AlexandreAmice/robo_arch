"""Shared protection setup with synthetic assets and no concrete device imports."""

import os
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from robo_arch.core.config.declarations import (
    ObjectInstance,
    Pose,
    RobotInstance,
    RobotSystem,
    SceneConfiguration,
    SensorInstance,
)
from robo_arch.core.controllers.cbf import geometry
from robo_arch.core.controllers.cbf.assembly import resolve_geometry
from robo_arch.core.controllers.cbf.config import ProtectionParameters


@pytest.fixture
def profile(tmp_path, monkeypatch):
    (tmp_path / "model.urdf").write_text("""<robot name="synthetic"><link name="body">
      <collision><origin xyz="1 0 0"/><geometry><box size="0.2 0.2 0.2"/>
      </geometry></collision></link></robot>""")
    (tmp_path / "protection.yaml").write_text(
        "asset: package://robo_arch/synthetic/model.urdf\ncell_size: 1.0\n"
    )
    monkeypatch.setattr(
        geometry,
        "resolve_resource",
        lambda resource: tmp_path / resource.rsplit("/", 1)[1],
    )
    return "package://robo_arch/synthetic/protection.yaml"


@pytest.fixture
def scene(tmp_path):
    return SceneConfiguration(
        robot_system=RobotSystem(
            name="",
            source=tmp_path / "system.yaml",
            pose=Pose(),
            robots=(
                RobotInstance(
                    name="arm",
                    model="synthetic_arm",
                    pose=Pose(),
                    initial_positions=None,
                ),
            ),
            sensors=(
                SensorInstance(
                    name="camera",
                    model="synthetic_sensor",
                    parent="arm/body",
                    pose=Pose(),
                    parameters={},
                ),
            ),
        ),
        sensors_enabled=False,
        objects=(
            ObjectInstance(
                name="obstacle",
                model="synthetic_object",
                pose=Pose(translation=(1, 2, 3), rpy=(0, 0, np.pi / 2)),
            ),
        ),
    )


def selection(profile, **changes):
    return ProtectionParameters.model_validate(
        {
            # Intentionally not alphabetical: preserve existing profile/pair order.
            "profiles": {"obstacle": profile, "camera": profile, "arm": profile},
            "protected": ("camera",),
            **changes,
        }
    )


def test_world_object_pose_and_existing_pair_order(scene, profile):
    result = resolve_geometry(scene, selection(profile))
    assert tuple(s.name for s in result.spheres) == (
        "obstacle/body/0/0",
        "camera/body/0/0",
        "arm/body/0/0",
    )
    obstacle, camera, arm = result.spheres
    assert obstacle.frame == "world"
    np.testing.assert_allclose(obstacle.center, [1, 3, 3], atol=1e-12)
    assert camera.frame == "camera/body" and arm.frame == "arm/body"
    np.testing.assert_allclose(camera.center, [1, 0, 0], atol=1e-12)
    assert [(pair.first, pair.second) for pair in result.pairs] == [
        (camera.name, obstacle.name),
        (arm.name, camera.name),
    ]
    assert all(pair.margin == 0.01 for pair in result.pairs)
    assert not result.planes and not result.plane_pairs


def test_fixed_object_profile_rejects_offset_links(scene, profile, tmp_path):
    # Object pose places its base at x=1. The tip's origin adds another 2 m;
    # treating this link-local covering as base-local would silently misplace it.
    (tmp_path / "model.urdf").write_text("""<robot name="fixture">
      <link name="base"/><link name="tip"><collision>
        <geometry><box size="0.2 0.2 0.2"/></geometry>
      </collision></link><joint name="offset" type="fixed">
        <parent link="base"/><child link="tip"/><origin xyz="2 0 0"/>
      </joint></robot>""")
    with pytest.raises(ValueError, match="Fixed-object.*single-link"):
        resolve_geometry(scene, selection(profile))


def test_ground_selection_matches_instance_not_name_prefix(scene, profile):
    selected = selection(
        profile,
        profiles={"camera": profile, "camera/other": profile},
        protected=("camera",),
    )
    result = resolve_geometry(scene, selected, ground=True)
    assert {pair.sphere for pair in result.plane_pairs} == {"camera/body/0/0"}
    assert any(s.name == "camera/other/body/0/0" for s in result.spheres)


def test_mount_exclusion_is_explicit_and_does_not_remove_other_pairs(scene, profile):
    result = resolve_geometry(
        scene, selection(profile, exclude_frames=(("camera/body", "arm/body"),))
    )
    assert len(result.pairs) == 1
    assert result.pairs[0].first.startswith("camera/")
    assert result.pairs[0].second.startswith("obstacle/")


@pytest.mark.parametrize(
    ("exclusions", "message"),
    [
        ((("camera/body", "arm/body"), ("arm/body", "camera/body")), "Duplicate"),
        ((("camera/body", "missing/body"),), "covered frames"),
        ((("camera/body", "camera/body"),), "distinct"),
    ],
)
def test_invalid_mount_exclusions_fail(scene, profile, exclusions, message):
    with pytest.raises(ValueError, match=message):
        resolve_geometry(scene, selection(profile, exclude_frames=exclusions))


@pytest.mark.parametrize("protected", ["camera", "obstacle"])
def test_ground_protects_selected_devices_or_fixed_objects(scene, profile, protected):
    selected = selection(profile, protected=(protected,), margin=0.025)
    result = resolve_geometry(scene, selected, ground=True)
    assert len(result.planes) == len(result.plane_pairs) == 1
    assert result.planes[0].normal == (0, 0, 1)
    assert result.planes[0].offset == 0
    assert result.plane_pairs[0].sphere == f"{protected}/body/0/0"
    assert result.plane_pairs[0].margin == 0.025
    assert all(
        pair.first.startswith(protected + "/")
        or pair.second.startswith(protected + "/")
        for pair in result.pairs
    )


def test_resolution_does_not_import_simulators_or_concrete_packages(tmp_path, profile):
    # Fresh process ensures cached imports cannot hide an SDK/device dependency.
    code = """
import importlib.abc
import sys
from pathlib import Path

class RejectDependencies(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"pydrake", "omni", "isaacsim"} or fullname.startswith(
            ("robo_arch.robots", "robo_arch.sensors", "robo_arch.scenarios", "robo_arch.robot_system")
        ):
            raise RuntimeError(f"Protection setup imported a concrete dependency: {fullname}")

sys.meta_path.insert(0, RejectDependencies())
from robo_arch.core.config.declarations import ObjectInstance, Pose, RobotSystem, SceneConfiguration
from robo_arch.core.controllers.cbf import geometry
from robo_arch.core.controllers.cbf.assembly import resolve_geometry
from robo_arch.core.controllers.cbf.config import ProtectionParameters

root = Path(sys.argv[1])
geometry.resolve_resource = lambda resource: root / resource.rsplit("/", 1)[1]
scene = SceneConfiguration(
    robot_system=RobotSystem(name="", source=root/"unused.yaml", pose=Pose()),
    sensors_enabled=False,
    objects=(ObjectInstance(name="fixture", model="synthetic", pose=Pose(rpy=(0, 0, 1.0))),),
)
parameters = ProtectionParameters(
    profiles={"fixture": "package://robo_arch/synthetic/protection.yaml"}, protected=("fixture",),
)
resolved = resolve_geometry(scene, parameters, ground=True)
assert resolved.spheres[0].frame == "world"
assert len(resolved.plane_pairs) == 1
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(Path(path).resolve()) for path in sys.path if path
    )
    subprocess.run(
        [sys.executable, "-c", code, str(tmp_path)],
        cwd=tmp_path,
        env=environment,
        check=True,
    )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
