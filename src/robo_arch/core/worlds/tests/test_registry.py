"""Lazy factory imports and SDK-independent configuration inspection."""

import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from pydantic import Field, ValidationError

from robo_arch.core.config.loading import (
    AutonomySelection,
    ObjectInstance,
    Pose,
    RobotInstance,
    RunConfiguration,
    SensorInstance,
    TaskSelection,
)
from robo_arch.core.config.parameters import Parameters
from robo_arch.core.config.worlds import DrakeWorld, RealWorld
from robo_arch.core.worlds.registry import FactoryReference, discover


class TrackingParameters(Parameters):
    gain: float = Field(default=1.0, gt=0)


def _run() -> RunConfiguration:
    return RunConfiguration(
        source=Path("unused.yaml"),
        world_config=DrakeWorld(),
        duration=1.0,
        robots=(
            RobotInstance(name="arm", model="ur7e", poses=(), initial_positions=None),
        ),
        sensors=(
            SensorInstance(
                name="camera",
                model="ideal_camera",
                parent="arm/tool0",
                pose=Pose(),
                parameters={},
            ),
        ),
        objects=(ObjectInstance(name="target", model="box", pose=Pose()),),
        task=TaskSelection(type="tracking"),
        autonomy=AutonomySelection(controller="joint_tracking"),
        resources=(),
    )


def test_discovery_loads_only_selected_definitions_and_deduplicates_models():
    run = _run()
    run = replace(run, robots=(*run.robots, replace(run.robots[0], name="other")))
    registry = discover(run)
    assert tuple(registry.robots) == ("ur7e",)
    assert len(registry.robots["ur7e"].joints) == 6
    assert tuple(registry.sensors) == ("ideal_camera",)
    assert registry.objects["box"].resource == "model.sdf"


@pytest.mark.parametrize("name", ["../ur7e", "ur7e.drake", "os:path", ""])
def test_discovery_rejects_model_import_paths(name):
    run = _run()
    with pytest.raises(ValueError, match="Invalid robots model identifier"):
        discover(replace(run, robots=(replace(run.robots[0], model=name),)))


def test_unknown_device_and_unsupported_world_fail_before_factory_loading():
    run = _run()
    with pytest.raises(ValueError, match="Unknown robots model: unknown_robot"):
        discover(replace(run, robots=(replace(run.robots[0], model="unknown_robot"),)))
    with pytest.raises(ValueError, match="Robot arm has no real implementation"):
        discover(replace(run, world_config=RealWorld()))
    with pytest.raises(ValidationError):
        discover(
            replace(
                run,
                sensors=(replace(run.sensors[0], parameters={"width": 0}),),
            )
        )


def test_parameter_schema_defaults_constraints_and_unknown_fields():
    assert TrackingParameters.model_validate({}).gain == 1.0
    assert TrackingParameters.model_validate({"gain": 2.0}).gain == 2.0
    with pytest.raises(ValidationError, match="greater than 0"):
        TrackingParameters.model_validate({"gain": -1})
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        TrackingParameters.model_validate({"gaim": 2.0})


def test_factory_import_is_explicit_and_checks_callable(tmp_path, monkeypatch):
    module = "declaration_test_factory"
    (tmp_path / f"{module}.py").write_text(
        "def build(value):\n    return value\nnot_a_factory = 1\n"
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    reference = FactoryReference(module=module, attribute="build")
    assert module not in sys.modules
    assert reference.load()("value") == "value"
    with pytest.raises(TypeError, match="not callable"):
        FactoryReference(module=module, attribute="not_a_factory").load()
    # Avoid leaving a module pointing into an expired temporary directory.
    monkeypatch.delitem(sys.modules, module)


def test_missing_factory_is_not_silently_substituted():
    reference = FactoryReference(module="robo_arch_missing_factory", attribute="build")
    with pytest.raises(ModuleNotFoundError):
        reference.load()


def test_declarations_import_in_fresh_process_without_simulator_sdks():
    script = f"""
import sys
sys.path[:] = {sys.path!r}
class RejectSDK:
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in {{'pydrake', 'isaacsim', 'omni', 'pxr', 'rclpy'}}:
            raise AssertionError('Unexpected SDK import: ' + fullname)
sys.meta_path.insert(0, RejectSDK())
from robo_arch.core.worlds.registry import FactoryReference
import robo_arch.core.config.loading
import robo_arch.core.worlds.selection
from robo_arch.robots.ur7e.definition import DEFINITION as robot
from robo_arch.sensors.ideal_camera.definition import DEFINITION as sensor
from robo_arch.objects.box.definition import DEFINITION as obj
assert len(robot.joints) == 6
assert sensor.parameter_schema.model_validate({{}}).width == 64
assert obj.resource == 'model.sdf'
reference = FactoryReference(module='isaacsim.future_wrapper', attribute='build')
assert reference.module == 'isaacsim.future_wrapper'
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
