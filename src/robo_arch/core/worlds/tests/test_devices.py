"""Package lookup, explicit adapter entry points and SDK-independent metadata."""

import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import pytest
from pydantic import Field, ValidationError

from robo_arch.core.config.declarations import (
    AutonomySelection,
    ObjectInstance,
    Pose,
    RobotInstance,
    RobotSystem,
    RunConfiguration,
    SensorInstance,
    TaskSelection,
)
from robo_arch.core.config.parameters import Parameters
from robo_arch.core.worlds.devices import load_definitions, load_device_module


class TrackingParameters(Parameters):
    gain: float = Field(default=1.0, gt=0)


def _run() -> RunConfiguration:
    return RunConfiguration(
        source=Path("unused.yaml"),
        world="drake",
        duration=1.0,
        time_step=0.001,
        robot_system=RobotSystem(
            name="",
            source=Path("system.yaml"),
            pose=Pose(),
            robots=(
                RobotInstance(
                    name="arm", model="ur7e", pose=Pose(), initial_positions=None
                ),
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
        ),
        sensors_enabled=True,
        objects=(ObjectInstance(name="target", model="box", pose=Pose()),),
        task=TaskSelection(type="tracking"),
        autonomy=AutonomySelection(controller="joint_tracking"),
    )


def test_discovery_loads_only_selected_definitions_and_deduplicates_models():
    run = _run()
    system = run.robot_system
    run = replace(
        run,
        robot_system=RobotSystem(
            name="",
            source=Path("pair.yaml"),
            pose=Pose(),
            systems=(replace(system, name="left"), replace(system, name="right")),
        ),
    )
    definitions = load_definitions(run)
    assert tuple(definitions.robots) == ("ur7e",)
    assert len(definitions.robots["ur7e"].joints) == 6
    assert tuple(definitions.sensors) == ("ideal_camera",)
    assert definitions.objects["box"].resource == "model.sdf"


@pytest.mark.parametrize("name", ["../ur7e", "ur7e.drake", "os:path", ""])
def test_discovery_rejects_model_import_paths(name):
    run = _run()
    with pytest.raises(ValueError, match="Invalid robots model identifier"):
        load_definitions(
            replace(
                run,
                robot_system=replace(
                    run.robot_system,
                    robots=(replace(run.robot_system.robots[0], model=name),),
                ),
            )
        )


def test_unknown_device_and_unsupported_world_fail_before_adapter_loading():
    run = _run()
    with pytest.raises(ModuleNotFoundError, match="robo_arch.robots.unknown_robot"):
        load_definitions(
            replace(
                run,
                robot_system=replace(
                    run.robot_system,
                    robots=(
                        replace(run.robot_system.robots[0], model="unknown_robot"),
                    ),
                ),
            )
        )
    with pytest.raises(ValueError, match="Robot arm has no real implementation"):
        load_definitions(replace(run, world="real"))
    with pytest.raises(ValueError, match="Sensor camera has no isaac implementation"):
        load_definitions(replace(run, world="isaac"))
    definitions = load_definitions(replace(run, world="isaac", sensors_enabled=False))
    assert definitions.sensors == {}
    assert tuple(definitions.robots) == ("ur7e",)
    with pytest.raises(ValidationError):
        load_definitions(
            replace(
                run,
                robot_system=replace(
                    run.robot_system,
                    sensors=(
                        replace(run.robot_system.sensors[0], parameters={"width": 0}),
                    ),
                ),
            )
        )


def test_parameter_schema_defaults_constraints_and_unknown_fields():
    assert TrackingParameters.model_validate({}).gain == 1.0
    assert TrackingParameters.model_validate({"gain": 2.0}).gain == 2.0
    with pytest.raises(ValidationError, match="greater than 0"):
        TrackingParameters.model_validate({"gain": -1})
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        TrackingParameters.model_validate({"gaim": 2.0})


def test_missing_device_adapter_is_not_substituted():
    with pytest.raises(ModuleNotFoundError, match="robo_arch.robots.ur7e.real"):
        load_device_module("robots", "ur7e", "real")


def test_device_module_rejects_import_paths():
    with pytest.raises(ValueError, match="Invalid device module identifier"):
        load_device_module("robots", "ur7e", "../drake")


def test_declarations_import_in_fresh_process_without_simulator_sdks():
    script = f"""
import sys
sys.path[:] = {sys.path!r}
class RejectSDK:
    def find_spec(self, fullname, path=None, target=None):
        if (fullname.split('.')[0] in {{'pydrake', 'isaacsim', 'omni', 'pxr', 'rclpy'}}
            or fullname.endswith(('.drake', '.isaac', '.real'))):
            raise AssertionError('Unexpected SDK import: ' + fullname)
sys.meta_path.insert(0, RejectSDK())
import robo_arch.core.config.declarations
assert 'yaml' not in sys.modules
import robo_arch.core.config.loading
from robo_arch.robots.ur7e.definition import describe as describe_robot
robot = describe_robot()
from robo_arch.sensors.ideal_camera.definition import describe as describe_sensor
sensor = describe_sensor()
from robo_arch.objects.box.definition import describe as describe_object
obj = describe_object()
assert len(robot.joints) == 6
assert sensor.parameter_schema.model_validate({{}}).width == 64
assert obj.resource == 'model.sdf'
assert robot.supported_worlds == ('drake', 'isaac')
assert sensor.supported_worlds == ('drake',)
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
