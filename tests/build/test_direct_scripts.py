"""Entry points expose help and resolve routing before loading simulator SDKs."""

import json
import subprocess
import sys
from importlib.resources import files

import pytest


@pytest.mark.parametrize(
    "scenario, operation",
    [
        ("arm_tracking", "run"),
        ("camera_protection", "run"),
        ("batched_reaching", "run"),
        ("camera_protection", "benchmark"),
        ("batched_reaching", "benchmark"),
    ],
)
def test_direct_help_without_sdk_or_build(scenario, operation):
    script = str(files("robo_arch") / f"scenarios/{scenario}/{operation}.py")
    program = f"""
import sys, runpy, subprocess
sys.path[:] = {sys.path!r}
class RejectSDK:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'drake', 'pydrake', 'isaacsim', 'isaaclab', 'torch', 'omni', 'pxr', 'robo_arch_native'}}:
            raise AssertionError('SDK imported for help: ' + fullname)
sys.meta_path.insert(0, RejectSDK())
def reject(*a, **kw):
    raise AssertionError('help must not build or launch')
subprocess.run = subprocess.check_output = reject
sys.argv = [{script!r}, '--help']
runpy.run_path({script!r}, run_name='__main__')
"""
    result = subprocess.run(
        [sys.executable, "-c", program], text=True, capture_output=True
    )
    assert result.returncode == 0, result.stderr
    assert "usage:" in result.stdout


@pytest.mark.parametrize(
    "scenario, options, expected",
    [
        ("arm_tracking", ["--world", "isaac"], "isaac"),
        (
            "camera_protection",
            [
                "--world-config",
                "package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml",
                "--backend",
                "torch_moreau",
            ],
            "isaac",
        ),
        ("batched_reaching", [], "isaac"),
    ],
)
def test_configuration_routes_before_runtime(scenario, options, expected):
    script = str(files("robo_arch") / f"scenarios/{scenario}/run.py")
    program = f"""
import sys, runpy, json
sys.path[:] = {sys.path!r}
from robo_arch.core.worlds import launch
class RejectSDK:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'pydrake', 'isaacsim', 'isaaclab', 'torch', 'omni', 'pxr', 'robo_arch_native'}}:
            raise AssertionError('SDK imported before routing: ' + fullname)
sys.meta_path.insert(0, RejectSDK())
def prepared(profile, **kwargs):
    print(json.dumps({{'profile': profile, **kwargs}}))
    raise SystemExit(0)
launch.prepare = prepared
sys.argv = [{script!r}, *{options!r}]
runpy.run_path({script!r}, run_name='__main__')
"""
    result = subprocess.run(
        [sys.executable, "-c", program], text=True, capture_output=True
    )
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)["profile"] == expected


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
