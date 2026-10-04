"""Build native algorithms and launch scenarios in their selected environment."""

import argparse
import hashlib
import json
import os
import subprocess
import zipfile
from pathlib import Path

from robo_arch.core.config.loading import load_run, load_world

ROOT = Path(__file__).resolve().parents[1]
TARGET = "//src/robo_arch/core/controllers/joint_pd:wheel"


def install(profile: str) -> Path:
    """Install into an existing uv environment, comparing actual extension bytes."""
    environment = ROOT / ("third_party/isaac/.venv" if profile == "isaac" else ".venv")
    python = environment / "bin/python"
    if not python.exists():
        project = " --project third_party/isaac" if profile == "isaac" else ""
        raise FileNotFoundError(
            f"Create the environment first: uv sync{project} --locked"
        )
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


def scenario_command(command: list[str]) -> tuple[str, list[str]]:
    """Select the interpreter without changing the scenario's runtime options."""
    parser = argparse.ArgumentParser(
        prog="tools/dev.py run arm_tracking",
        description="Run arm tracking in the environment selected by its world.",
        epilog=(
            "Other scenario options, including --visualization, --no-sensors, "
            "--record and --metadata, are forwarded unchanged."
        ),
        allow_abbrev=False,
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument("--world", choices=("drake", "isaac"))
    selection.add_argument("--world-config", help="Complete world profile file or URI")
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument(
        "--run",
        default="package://robo_arch/scenarios/arm_tracking/scenario.yaml",
        help="Scenario file or package URI",
    )
    inputs.add_argument("--inspect", type=Path, help="Restore a saved run report")
    args, _ = parser.parse_known_args(command)
    if args.world is not None:
        profile = args.world
    elif args.world_config is not None:
        profile = load_world(args.world_config).type
    elif args.inspect is not None:
        report = json.loads(args.inspect.read_text(encoding="utf-8"))
        profile = report["configuration"]["world_config"]["type"]
    else:
        profile = load_run(args.run).world
    if profile not in {"drake", "isaac"}:
        parser.error(f"No development environment for world {profile!r}")
    return profile, ["-m", "robo_arch.scenarios.arm_tracking.run", *command]


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    operations = parser.add_subparsers(dest="operation", required=True)
    native = operations.add_parser("native", help="Build/install native algorithms")
    native.add_argument("--profile", choices=("drake", "isaac"), required=True)
    run = operations.add_parser("run", help="Launch a scenario or Python command")
    run.add_argument(
        "--profile", choices=("drake", "isaac"), help="Environment for a Python command"
    )
    run.add_argument(
        "command",
        nargs=argparse.REMAINDER,
        help="arm_tracking [options], or --profile <world> -- python <arguments>",
    )
    args = parser.parse_args(argv)
    if args.operation == "native":
        install(args.profile)
        return
    command = args.command
    if command[:1] == ["--"]:
        command = command[1:]
    if not command:
        parser.error(
            "run requires arm_tracking [options] or --profile <world> -- python"
        )
    # Keep relative input/output paths rooted at the checkout, as in the generic runner.
    os.chdir(ROOT)
    if args.profile is not None:
        if command[0] != "python":
            parser.error("--profile requires a Python command: -- python <arguments>")
        profile, command = args.profile, command[1:]
    elif command[0] == "arm_tracking":
        profile, command = scenario_command(command[1:])
    else:
        parser.error(
            f"Unknown scenario {command[0]!r}; supported scenario: arm_tracking"
        )
    python = install(profile)
    # Replace the launcher so the runtime receives Ctrl-C and owns its cleanup/exit code.
    os.execv(str(python), [str(python), *command])


if __name__ == "__main__":
    main()
