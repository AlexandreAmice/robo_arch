"""Native payload freshness and build failures control whether a launch can proceed."""

import hashlib
import json
import subprocess
import sys
import zipfile

import pytest

from tools.native import install


@pytest.fixture
def build(tmp_path, monkeypatch):
    monkeypatch.setattr(install, "ROOT", tmp_path)
    python = tmp_path / ".venv/bin/python"
    python.parent.mkdir(parents=True)
    python.touch()
    wheel = tmp_path / "native.whl"
    payload = {
        "robo_arch_native/_joint_pd.so": b"extension",
        "robo_arch_native/__init__.py": b"package",
        "robo_arch_native/build.json": json.dumps(
            {"extensions": ["robo_arch_native._example", "robo_arch_native._second"]}
        ).encode(),
    }
    with zipfile.ZipFile(wheel, "w") as archive:
        for name, data in payload.items():
            archive.writestr(name, data)
    return tmp_path, {
        name: hashlib.sha256(data).hexdigest() for name, data in payload.items()
    }


@pytest.mark.parametrize(
    "state", ["missing", "changed", "same", "missing_package_file"]
)
def test_install_only_when_payload_differs(build, monkeypatch, state):
    root, expected = build
    installed = dict(expected)
    if state == "missing":
        installed = None
    elif state == "changed":
        installed["robo_arch_native/_joint_pd.so"] = "old"
    elif state == "missing_package_file":
        installed.pop("robo_arch_native/__init__.py")
    outputs = iter(
        ["(3, 12)\nLinux x86_64\ncpython\n", "native.whl", json.dumps(installed)]
    )
    monkeypatch.setattr(
        install.subprocess, "check_output", lambda *a, **kw: next(outputs)
    )
    calls = []
    monkeypatch.setattr(
        install.subprocess, "run", lambda command, **kw: calls.append(command)
    )
    assert install.install("drake") == root / ".venv/bin/python"
    assert calls[0] == ["bazel", "build", install.TARGET]
    assert any(command[:2] == ["uv", "pip"] for command in calls) == (state != "same")
    assert json.loads(calls[-1][-1]) == [
        "robo_arch_native._example",
        "robo_arch_native._second",
    ]


@pytest.mark.parametrize("stage", ["build", "install"])
def test_failure_stops_before_import(build, monkeypatch, stage):
    outputs = iter(["(3, 12)\nLinux x86_64\ncpython\n", "native.whl", "null"])
    monkeypatch.setattr(
        install.subprocess, "check_output", lambda *a, **kw: next(outputs)
    )
    calls = []

    def run(command, **kw):
        calls.append(command)
        if command[0] == ("bazel" if stage == "build" else "uv"):
            raise subprocess.CalledProcessError(1, command)

    monkeypatch.setattr(install.subprocess, "run", run)
    with pytest.raises(subprocess.CalledProcessError):
        install.install("drake")
    assert all(command[1:2] != ["-c"] for command in calls)


def test_incompatible_python_never_builds(build, monkeypatch):
    monkeypatch.setattr(
        install.subprocess,
        "check_output",
        lambda *a, **kw: "(3, 13)\nLinux x86_64\ncpython\n",
    )
    monkeypatch.setattr(
        install.subprocess, "run", lambda *a, **kw: pytest.fail("build")
    )
    with pytest.raises(RuntimeError, match="CPython 3.12"):
        install.install("drake")


@pytest.mark.parametrize("missing", [None, "__init__.py", "_example.so", "data.bin"])
def test_real_payload_probe_detects_missing_files(tmp_path, missing):
    package = tmp_path / "robo_arch_native"
    package.mkdir()
    contents = {
        "__init__.py": b"raise AssertionError('probe must not import native package')",
        "_example.so": b"native bytes",
        "data.bin": b"runtime resource",
    }
    for name, content in contents.items():
        if name != missing:
            (package / name).write_bytes(content)
    script = (
        "import sys; sys.path[:] = [sys.argv[1], "
        "*[p for p in sys.path if 'site-packages' not in p]]\n" + install.PAYLOAD_PROBE
    )
    result = json.loads(
        subprocess.check_output(
            [sys.executable, "-I", "-c", script, str(tmp_path)], text=True
        )
    )
    expected = {
        f"robo_arch_native/{name}": hashlib.sha256(content).hexdigest()
        for name, content in contents.items()
    }
    if missing == "__init__.py":
        assert result is None
    elif missing is not None:
        assert result != expected
        assert result == {
            name: value
            for name, value in expected.items()
            if name.split("/")[-1] != missing
        }
    else:
        assert result == expected


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
