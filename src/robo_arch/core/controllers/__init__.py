"""Robot-independent control algorithms and their world adapters."""

from importlib import import_module
from types import ModuleType


def definition(name: str) -> ModuleType:
    """Resolve a controller declaration without importing its simulator."""
    if not name.isidentifier() or name.startswith("_"):
        raise ValueError(f"Invalid controller name: {name!r}")
    module = f"robo_arch.core.controllers.{name}.definition"
    try:
        return import_module(module)
    except ModuleNotFoundError as error:
        if error.name == module or module.startswith(f"{error.name}."):
            raise ValueError(f"Unknown controller: {name}") from error
        raise
