"""Real scenario execution boundary; RViz remains an independent viewer."""

from typing import NoReturn

from robo_arch.core.config.declarations import RunConfiguration


def run_scenario(run: RunConfiguration) -> NoReturn:
    """No hardware commands or device processes are launched."""
    raise NotImplementedError("Real scenario execution requires a hardware runner")
