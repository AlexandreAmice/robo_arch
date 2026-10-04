"""Build/install native algorithms and optionally launch a fresh local process."""

import argparse
import hashlib
import json
import os
import subprocess
import sys
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


def scenario_environment(scenario: str, command: list[str]) -> tuple[str, bool]:
    """Resolve environment and display needs using SDK-independent declarations."""
    from pydantic import TypeAdapter

    from robo_arch.core.config.declarations import RunConfiguration
    from robo_arch.core.config.loading import load_run, load_world
    from robo_arch.core.config.worlds import parse_world

    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--inspect", type=Path)
    parser.add_argument("--run" if scenario == "arm_tracking" else "--config")
    parser.add_argument("--world")
    parser.add_argument("--world-config")
    parser.add_argument("--visualization")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--live", action="store_true")
    args, _ = parser.parse_known_args(command)
    if args.inspect:
        report = json.loads(args.inspect.read_text(encoding="utf-8"))
        run = TypeAdapter(RunConfiguration).validate_python(report["configuration"])
    else:
        resource = args.run if scenario == "arm_tracking" else args.config
        run = load_run(
            resource or f"package://robo_arch/scenarios/{scenario}/scenario.yaml"
        )
    world = run.world_config
    if args.world:
        world = parse_world({"type": args.world})
    elif args.world_config:
        world = load_world(args.world_config)
    if world.type not in {"drake", "isaac"}:
        raise ValueError(f"No local runtime profile for world {world.type!r}")
    mode = args.visualization or world.visualization.mode
    live = (args.live or mode in {"live", "live_and_record"}) and not args.headless
    return world.type, live


def launch_environment(profile: str, *, live: bool) -> dict[str, str]:
    """Keep vendor startup settings local to the child process."""
    environment = os.environ.copy()
    if profile == "isaac":
        environment.setdefault("OMNI_KIT_ACCEPT_EULA", "YES")
        if not live:
            environment.pop("DISPLAY", None)
            environment.pop("WAYLAND_DISPLAY", None)
    return environment


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=(
            "Examples: run arm_tracking; run batched_reaching --backend newton "
            "--live --hold; benchmark batched_reaching. "
            "Append --help after the scenario name for its options."
        ),
        allow_abbrev=False,
    )
    parser.add_argument("operation", choices=("native", "run", "benchmark"))
    parser.add_argument("--profile", choices=("drake", "isaac"))
    # Leave scenario flags (including --help) to the scenario's own parser.
    argv = sys.argv[1:] if argv is None else argv
    separator = next(
        (
            i
            for i, value in enumerate(argv)
            if value in {"arm_tracking", "batched_reaching", "--"}
        ),
        len(argv),
    )
    args, command = parser.parse_known_args(argv[:separator])
    command.extend(argv[separator:])
    if command[:1] == ["--"]:
        command = command[1:]
    if args.operation == "native" and command:
        parser.error("native accepts no child command")
    profile, environment = args.profile, None
    if args.operation != "native" and not command:
        parser.error("choose arm_tracking or batched_reaching")
    if command and command[0] in {"arm_tracking", "batched_reaching"}:
        scenario, *options = command
        if args.operation == "benchmark":
            if scenario != "batched_reaching":
                parser.error("benchmark supports batched_reaching")
            selected, live = "isaac", False
        else:
            selected, live = scenario_environment(scenario, options)
        if profile is not None and profile != selected:
            parser.error(
                f"scenario selects {selected}, conflicting with --profile {profile}"
            )
        profile = selected
        environment = launch_environment(profile, live=live)
        command = [
            "python",
            "-m",
            f"robo_arch.scenarios.{scenario}.{args.operation}",
            *options,
        ]
    elif command and (command[0] != "python" or args.operation != "run"):
        parser.error(
            "choose arm_tracking or batched_reaching, "
            "or use --profile <world> -- python <arguments>"
        )
    if profile is None:
        parser.error("native and raw Python commands require --profile")
    python = install(profile)
    if command:
        subprocess.run(
            [str(python), *command[1:]], cwd=ROOT, env=environment, check=True
        )


if __name__ == "__main__":
    main()
