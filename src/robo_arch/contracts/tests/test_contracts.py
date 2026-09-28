"""Meaningful declaration checks; these do not test S0 compatibility decisions."""

import subprocess
import sys
from dataclasses import replace

import pytest
from pydantic import Field, ValidationError

from robo_arch.contracts.components import FactoryReference, Parameters
from robo_arch.contracts.ports import (
    ArrayOwnership,
    CommandMode,
    PortDescription,
    TimestampConvention,
)


class TrackingParameters(Parameters):
    gain: float = Field(default=1.0, gt=0)


def test_parameter_schema_defaults_constraints_and_unknown_fields():
    assert TrackingParameters.model_validate({}).gain == 1.0
    assert TrackingParameters.model_validate({"gain": 2.0}).gain == 2.0
    with pytest.raises(ValidationError, match="greater than 0"):
        TrackingParameters.model_validate({"gain": -1})
    with pytest.raises(ValidationError, match="Extra inputs are not permitted"):
        TrackingParameters.model_validate({"gaim": 2.0})


def test_command_meaning_is_distinct_from_shape():
    position = PortDescription(
        quantity="joint_position_command",
        units="rad",
        dimensions=("robot.num_joints",),
        timestamp=TimestampConvention(clock="world", event="command_application"),
        ownership=ArrayOwnership.BORROWED_READ_ONLY,
        robot="ur7e",
        joints=("shoulder_pan_joint", "shoulder_lift_joint"),
        command_mode=CommandMode.POSITION,
    )
    effort = replace(
        position,
        quantity="joint_effort_command",
        units="N*m",
        command_mode=CommandMode.EFFORT,
    )
    assert position.dimensions == effort.dimensions
    assert position != effort
    assert position != replace(position, joints=tuple(reversed(position.joints)))


def test_factory_import_is_explicit_and_checks_callable(tmp_path, monkeypatch):
    module = "i0_test_factory"
    (tmp_path / f"{module}.py").write_text(
        "def build(*, instance, context):\n"
        "    return (instance, context)\n"
        "not_a_factory = 1\n"
    )
    monkeypatch.syspath_prepend(str(tmp_path))
    reference = FactoryReference(module=module, attribute="build")
    assert module not in sys.modules
    assert reference.load()(instance="controller", context="world") == (
        "controller",
        "world",
    )
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
from robo_arch.contracts.components import FactoryReference
import robo_arch.contracts.ports
import robo_arch.config.descriptions
import robo_arch.config.resolved
import robo_arch.config.world
reference = FactoryReference(module='isaacsim.future_wrapper', attribute='build')
assert reference.module == 'isaacsim.future_wrapper'
"""
    result = subprocess.run(
        [sys.executable, "-c", script], capture_output=True, text=True, check=False
    )
    assert result.returncode == 0, result.stderr


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
