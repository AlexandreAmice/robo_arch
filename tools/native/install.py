"""Incrementally build and install the local native wheel into a uv environment."""

import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TARGET = "//src/robo_arch/core/controllers/joint_pd:wheel"


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
    if check.splitlines() != ["(3, 12)", "Linux x86_64", "cpython"]:
        raise RuntimeError("Native profile requires CPython 3.12 on Linux x86_64")
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
    probe = (
        "import importlib.util,hashlib,json; from pathlib import Path; "
        "s=importlib.util.find_spec('robo_arch_native'); "
        "p=Path(s.origin).parent if s else None; "
        "print(json.dumps({ 'robo_arch_native/'+f.relative_to(p).as_posix(): "
        "hashlib.sha256(f.read_bytes()).hexdigest() for f in p.rglob('*') "
        "if f.is_file() and '__pycache__' not in f.parts } if p else None))"
    )
    installed = json.loads(
        subprocess.check_output([str(python), "-c", probe], text=True)
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
    subprocess.run([str(python), "-c", "import robo_arch_native._joint_pd"], check=True)
    return python


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", choices=("drake", "isaac"), required=True)
    install(parser.parse_args().profile)


if __name__ == "__main__":
    main()
