"""Incrementally build and install the local native wheel into a uv environment."""

import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TARGET = "//tools/native:wheel"

PAYLOAD_PROBE = """
import importlib.util, hashlib, json
from pathlib import Path
spec = importlib.util.find_spec("robo_arch_native")
# A missing __init__.py leaves a namespace package, which must be reinstalled.
root = Path(spec.origin).parent if spec and spec.origin else None
print(json.dumps({
    "robo_arch_native/" + file.relative_to(root).as_posix():
        hashlib.sha256(file.read_bytes()).hexdigest()
    for file in root.rglob("*")
    if file.is_file() and "__pycache__" not in file.parts
} if root else None))
"""


def install(profile: str) -> Path:
    """Install into an existing uv environment, comparing actual package payloads."""
    environment = ROOT / ("third_party/isaac/.venv" if profile == "isaac" else ".venv")
    python = environment / "bin/python"
    if not python.exists():
        raise FileNotFoundError(f"Run uv sync for the {profile} environment first")
    check = subprocess.check_output(
        [
            str(python),
            "-c",
            "import sys,platform; print(sys.version_info[:2]); "
            "print(platform.system(), platform.machine()); print(sys.implementation.name)",
        ],
        text=True,
    )
    supported = [
        ["(3, 12)", "Linux x86_64", "cpython"],
        ["(3, 13)", "Darwin arm64", "cpython"],
    ]
    if check.splitlines() not in supported or (
        profile == "isaac" and check.splitlines() != supported[0]
    ):
        raise RuntimeError(
            "Native profile requires CPython 3.12 on Linux x86_64 or "
            "CPython 3.13 on macOS arm64 (Drake only)"
        )
    subprocess.run(["bazel", "build", TARGET], cwd=ROOT, check=True)
    output = subprocess.check_output(
        ["bazel", "cquery", TARGET, "--output=files"],
        cwd=ROOT,
        text=True,
    ).strip()
    wheel = ROOT / output
    with zipfile.ZipFile(wheel) as archive:
        expected = {
            name: hashlib.sha256(archive.read(name)).hexdigest()
            for name in archive.namelist()
            if name.startswith("robo_arch_native/") and not name.endswith("/")
        }
        modules = json.loads(archive.read("robo_arch_native/build.json"))["extensions"]
    installed = json.loads(
        subprocess.check_output([str(python), "-c", PAYLOAD_PROBE], text=True)
    )
    if installed != expected:
        subprocess.run(
            [
                "uv",
                "pip",
                "install",
                "--python",
                str(python),
                "--no-deps",
                "--reinstall",
                str(wheel),
            ],
            cwd=ROOT,
            check=True,
        )
    subprocess.run(
        [
            str(python),
            "-c",
            "import importlib,json,sys; "
            "[importlib.import_module(name) for name in json.loads(sys.argv[1])]",
            json.dumps(modules),
        ],
        check=True,
    )
    return python


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("drake", "isaac"), required=True)
    install(parser.parse_args().profile)


if __name__ == "__main__":
    main()
