"""Camera protection declarations and geometry load from packaged resources."""

import os
import subprocess
import sys
from pathlib import Path


def test_camera_resources_without_checkout_or_simulator(tmp_path):
    """Exercise actual resources from a foreign cwd while rejecting SDK imports."""
    code = """
import importlib.abc
import sys

class RejectSimulator(importlib.abc.MetaPathFinder):
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split(".")[0] in {"pydrake", "omni", "isaacsim"}:
            raise RuntimeError(f"Declaration imported simulator: {fullname}")

sys.meta_path.insert(0, RejectSimulator())

from robo_arch.core.config.loading import load_run
from robo_arch.core.controllers.cbf.geometry import load_sphere_profile
from robo_arch.core.controllers.cbf.assembly import resolve_geometry
from robo_arch.scenarios.camera_protection.configuration import parameters_for

run = load_run("package://robo_arch/scenarios/camera_protection/scenario.yaml")
control, task = parameters_for(run)
geometry = resolve_geometry(run.scene, control, ground=run.world_config.ground)
assert len(geometry.pairs) == 125
assert len(geometry.plane_pairs) == 3
assert geometry.planes[0].name == "ground"
assert len(run.robot_system.robots) == 1
cameras = run.robot_system.sensors
assert len(cameras) == len({sensor.name for sensor in cameras}) == 3
assert len({id(sensor) for sensor in cameras}) == 3
assert set(control.protected) == {sensor.name for sensor in cameras}
expected = {
    *(robot.name for robot in run.robot_system.robots),
    *(sensor.name for sensor in cameras),
    *(obj.name for obj in run.objects),
}
assert set(control.profiles) == expected
profiles = {
    name: load_sphere_profile(resource, name)
    for name, resource in control.profiles.items()
}
names = []
for owner, spheres in profiles.items():
    assert spheres
    for sphere in spheres:
        assert sphere.frame.startswith(owner + "/")
        assert sphere.radius > 0
        names.append(sphere.name)
assert len(names) == len(set(names))
assert task.retreat_time < run.duration
assert not any(name.split(".")[0] in {"pydrake", "omni", "isaacsim"}
               for name in sys.modules)
"""
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        str(Path(path).resolve()) for path in sys.path if path
    )
    subprocess.run(
        [sys.executable, "-c", code],
        cwd=tmp_path,
        env=environment,
        check=True,
    )


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__]))
