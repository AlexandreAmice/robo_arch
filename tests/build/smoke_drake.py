"""Minimal dependency compatibility check; this is not a world integration."""

import importlib.metadata

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource


def test_drake_environment() -> None:
    assert importlib.metadata.version("drake") == "1.57.0"
    builder = DiagramBuilder()
    builder.AddSystem(ConstantVectorSource([1.0]))
    assert builder.Build().num_input_ports() == 0


if __name__ == "__main__":
    import pytest

    raise SystemExit(pytest.main([__file__]))
