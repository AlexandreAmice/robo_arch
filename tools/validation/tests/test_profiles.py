"""Profile drift and requested provider failures must be actionable errors."""

import json
import subprocess
import sys

import pytest

from tools.validation.__main__ import PROBE
from tools.validation.profiles import ROOT, check_pins, metadata, profile, support_table


def test_profile_authority_and_generated_view():
    check_pins()
    assert (ROOT / "third_party/support.md").read_text() == support_table()
    for name in metadata()["profiles"]:
        entry = profile(name)
        assert (ROOT / entry["lock"]).is_file()


def test_requested_missing_provider_fails():
    result = subprocess.run(
        [
            sys.executable,
            "-c",
            PROBE,
            json.dumps(
                [
                    ".".join(map(str, sys.version_info[:3])),
                    ["unavailable_robo_sdk"],
                    False,
                ]
            ),
        ],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "Requested suite requires unavailable_robo_sdk" in result.stderr


def test_interpreter_drift_fails_before_sdk_probe():
    result = subprocess.run(
        [sys.executable, "-c", PROBE, json.dumps(["3.0.0", [], False])],
        capture_output=True,
        text=True,
    )
    assert result.returncode != 0
    assert "Interpreter" in result.stderr


def test_bazel_interpreter_drift_detected(tmp_path):
    (tmp_path / "third_party").mkdir()
    for name in (
        "third_party/compatibility.toml",
        ".python-version",
        ".python-version-macos",
        ".bazelrc",
    ):
        (tmp_path / name).write_bytes((ROOT / name).read_bytes())
    (tmp_path / "MODULE.bazel").write_text("")
    with pytest.raises(ValueError, match="pin must match"):
        check_pins(tmp_path)


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
