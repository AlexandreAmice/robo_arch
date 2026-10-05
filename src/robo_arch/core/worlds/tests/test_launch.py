"""Development launch preserves process semantics without importing simulators."""

import json
from pathlib import Path

import pytest

from robo_arch.core.worlds import launch


@pytest.fixture
def checkout(tmp_path, monkeypatch):
    for relative in (".venv/bin/python", "third_party/isaac/.venv/bin/python"):
        file = tmp_path / relative
        file.parent.mkdir(parents=True)
        file.touch()
    monkeypatch.setattr(launch, "development_root", lambda: tmp_path)
    monkeypatch.delenv(launch._HANDOFF, raising=False)
    return tmp_path


@pytest.mark.parametrize(
    "invocation",
    [
        ["relative/run.py", "--run", "input.yaml"],
        ["-m", "robo_arch.scenarios.arm_tracking.run", "--world", "isaac"],
    ],
)
def test_refresh_then_exec_and_consume_handoff(checkout, monkeypatch, invocation):
    events = []
    monkeypatch.setattr(launch.sys, "orig_argv", ["python", *invocation])
    monkeypatch.setattr(
        launch.subprocess, "run", lambda *a, **kw: events.append((a, kw))
    )
    monkeypatch.setattr(launch.os, "execve", lambda *a: events.append(a))
    before = Path.cwd()
    launch.prepare("isaac")
    assert Path.cwd() == before
    assert len(events) == 2
    assert events[0][0][0][-2:] == ["--profile", "isaac"]
    python, args, environment = events[1]
    assert python == str(checkout / "third_party/isaac/.venv/bin/python")
    assert args == [python, "-P", *invocation]
    monkeypatch.setattr(launch.sys, "orig_argv", args)
    assert environment["OMNI_KIT_ACCEPT_EULA"] == "YES"
    monkeypatch.setenv(launch._HANDOFF, environment[launch._HANDOFF])
    launch.prepare("isaac")
    assert len(events) == 2
    assert launch._HANDOFF not in launch.os.environ
    # A later launch in this process (or a benchmark child) must check again.
    launch.prepare("isaac")
    assert len(events) == 4


def test_inherited_marker_cannot_skip_refresh(checkout, monkeypatch):
    monkeypatch.setenv(launch._HANDOFF, json.dumps([-1, str(checkout), "drake", []]))

    def fail(*a, **kw):
        raise RuntimeError("build failed")

    monkeypatch.setattr(launch.subprocess, "run", fail)
    monkeypatch.setattr(launch.os, "execve", lambda *a: pytest.fail("stale launch"))
    with pytest.raises(RuntimeError, match="build failed"):
        launch.prepare("drake")


def test_missing_environment(checkout, monkeypatch):
    (checkout / ".venv/bin/python").unlink()
    monkeypatch.setattr(launch.subprocess, "run", lambda *a, **kw: pytest.fail("build"))
    with pytest.raises(FileNotFoundError, match="uv sync --locked"):
        launch.prepare("drake")


def test_installed_execution_does_not_build(monkeypatch):
    monkeypatch.setattr(launch, "development_root", lambda: None)
    monkeypatch.setenv("DISPLAY", ":1")
    monkeypatch.setattr(launch.subprocess, "run", lambda *a, **kw: pytest.fail("build"))
    launch.prepare("isaac")
    assert "DISPLAY" not in launch.os.environ


def test_worktree_root_is_source_owned(tmp_path, monkeypatch):
    (tmp_path / ".git").write_text("gitdir: elsewhere")
    (tmp_path / "MODULE.bazel").touch()
    source = tmp_path / "src/robo_arch/core/worlds/launch.py"
    monkeypatch.setattr(launch, "__file__", str(source))
    assert launch.development_root() == tmp_path
    (tmp_path / ".git").unlink()
    assert launch.development_root() is None


def test_inspection_selects_source_project_from_any_directory(checkout):
    script = checkout / "src/robo_arch/scenarios/example/run.py"
    command = launch.inspection_invocation(str(script))
    assert command == ["uv", "run", "--locked", "--project", str(checkout), str(script)]


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
