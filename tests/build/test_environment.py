"""The same smoke check runs under editable uv and Bazel's isolated interpreter."""

import sys

import pydantic
import yaml

import robo_arch


def test_core_environment() -> None:
    assert sys.version_info[:3] == (3, 12, 13)
    assert pydantic.__version__ == "2.13.5"
    assert yaml.safe_load("command: position") == {"command": "position"}
    assert robo_arch.__file__ is not None


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__]))
