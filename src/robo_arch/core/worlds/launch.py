"""Prepare direct development script execution before importing runtime SDKs."""

import json
import os
import subprocess
import sys
from pathlib import Path

_HANDOFF = "_ROBO_ARCH_PREPARED_PROCESS"


def development_root() -> Path | None:
    """Find this source installation's checkout, excluding installed/Bazel trees."""
    source = Path(__file__).absolute()
    root = source.parents[4]
    if (root / ".git").exists() and (root / "MODULE.bazel").is_file():
        return root.resolve()
    return None


def launch_environment(profile: str, *, live: bool) -> dict[str, str]:
    """Keep vendor startup settings local to the launched process."""
    environment = os.environ.copy()
    if profile == "isaac":
        environment.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        if not live:
            environment.pop("DISPLAY", None)
            environment.pop("WAYLAND_DISPLAY", None)
    return environment


def inspection_invocation(script: str) -> list[str]:
    """Return a replayable script command independent of the caller's directory."""
    root = development_root()
    source = str(Path(script).absolute())
    if root is None:
        return [sys.executable, "-P", source]
    return ["uv", "run", "--locked", "--project", str(root), source]


def prepare(profile: str, *, live: bool = False) -> None:
    """Refresh native code and re-exec this invocation in its world's environment.

    Call only from a script's main, after parsing/validation and before runtime
    imports. uv environments must already exist. Installed wheels and Bazel
    runfiles use their supplied native artifacts instead of building a checkout.
    """
    if profile not in {"drake", "isaac"}:
        raise ValueError(f"No local runtime profile for world {profile!r}")
    root = development_root()
    invocation = sys.orig_argv[1:]
    handoff = json.dumps([os.getpid(), str(root), profile, invocation])
    prepared = os.environ.pop(_HANDOFF, None)
    if prepared == handoff:
        return
    environment = launch_environment(profile, live=live)
    if root is None:
        # Direct files must not shadow SDK packages with sibling drake.py, etc.
        if (
            sys.path
            and Path(sys.path[0]).resolve() == Path(sys.argv[0]).resolve().parent
        ):
            sys.path.pop(0)
        os.environ.update(environment)
        if profile == "isaac" and not live:
            os.environ.pop("DISPLAY", None)
            os.environ.pop("WAYLAND_DISPLAY", None)
        return
    python = root / ("third_party/isaac/.venv" if profile == "isaac" else ".venv")
    python = python / "bin/python"
    if not python.is_file():
        command = "uv sync --locked"
        if profile == "isaac":
            command += " --project third_party/isaac"
        raise FileNotFoundError(f"Missing {python}; in {root}, run: {command}")
    subprocess.run(
        [sys.executable, str(root / "tools/native/install.py"), "--profile", profile],
        check=True,
    )
    if "-P" not in invocation:
        invocation = ["-P", *invocation]
    environment[_HANDOFF] = json.dumps([os.getpid(), str(root), profile, invocation])
    # exec preserves PID, exit status and signal handling; cwd stays with caller.
    os.execve(str(python), [str(python), *invocation], environment)
