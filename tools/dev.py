"""Build/install native algorithms and optionally launch a fresh local process."""

import argparse
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TARGET = "//src/robo_arch/core/controllers/joint_pd:wheel"


def install(profile: str) -> Path:
    """Install into an existing uv environment, comparing actual extension bytes."""
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
        expected = hashlib.sha256(
            archive.read("robo_arch_native/_joint_pd.so")
        ).hexdigest()
    probe = (
        "import importlib.util,hashlib,json; from pathlib import Path; "
        "s=importlib.util.find_spec('robo_arch_native'); "
        "p=Path(s.origin).parent/'_joint_pd.so' if s else None; "
        "print(json.dumps(hashlib.sha256(p.read_bytes()).hexdigest() "
        "if p and p.is_file() else None))"
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
    parser.add_argument("operation", choices=("native", "run"))
    parser.add_argument("--profile", choices=("drake", "isaac"), required=True)
    args, command = parser.parse_known_args()
    if command[:1] == ["--"]:
        command = command[1:]
    if args.operation == "native" and command:
        parser.error("native accepts no child command")
    if args.operation == "run" and not command:
        parser.error("run requires -- python <arguments>")
    if command and command[0] != "python":
        parser.error("run launches the selected environment: use -- python <arguments>")
    python = install(args.profile)
    if command:
        subprocess.run([str(python), *command[1:]], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
