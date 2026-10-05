"""One local entry point for explicitly selected runtime suites and host checks."""

import argparse
import json
import os
import subprocess
import tempfile
from pathlib import Path

from tools.validation.profiles import (
    ROOT,
    check_host,
    check_pins,
    metadata,
    profile,
    support_table,
    sync_command,
)

PROBE = """
import importlib.util, json, platform, sys
expected, providers, gpu = json.loads(sys.argv[1])
if platform.python_version() != expected:
    raise RuntimeError(f"Interpreter {platform.python_version()} != {expected}")
for name in providers:
    if importlib.util.find_spec(name) is None:
        raise ModuleNotFoundError(f"Requested suite requires {name}; sync selected profile")
if gpu:
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError("Requested suite requires CUDA; skipping is not a pass")
"""


def run_suites(name: str, suites: list[str], *, bazel: bool = False) -> None:
    record = profile(name)
    check_host(record)
    check_pins()
    if name == "ros":
        if suites != ["ros"] or bazel:
            raise ValueError("The ROS profile runs only its isolated process suite")
        from tools.validation.ros import run

        run()
        return
    subprocess.run(
        ["uv", "lock", "--check", "--project", record["project"]], cwd=ROOT, check=True
    )
    subprocess.run(sync_command(record, check=True), cwd=ROOT, check=True)
    python = ROOT / record["environment"] / "bin/python"
    if not python.is_file():
        raise FileNotFoundError(f"Missing {python}; run setup --sync first")
    with tempfile.TemporaryDirectory(prefix="robo-suite-") as directory:
        # A selected suite cannot turn a missing runtime into a green skipped run.
        plugin = Path(directory) / "required_suite.py"
        plugin.write_text(
            "def pytest_sessionfinish(session, exitstatus):\n"
            "    reporter = session.config.pluginmanager.getplugin('terminalreporter')\n"
            "    if reporter and reporter.stats.get('skipped'):\n"
            "        session.exitstatus = 1\n"
        )
        env = os.environ.copy()
        env["PYTHONPATH"] = str(ROOT) + os.pathsep + directory
        for selected in suites:
            suite = metadata()["suites"][selected]
            if name not in suite["profiles"]:
                raise ValueError(f"Suite {selected} is unavailable in profile {name}")
            if selected == "installed":
                from tools.validation.artifact import create

                bundle = Path(directory) / "bundle"
                create(name, bundle)
                subprocess.run(
                    [str(python), str(bundle / "verify_bundle.py")], check=True
                )
                continue
            if suite.get("native"):
                from tools.native.install import install

                install("isaac" if name == "isaac" else "drake")
            subprocess.run(
                [
                    str(python),
                    "-c",
                    PROBE,
                    json.dumps(
                        [
                            record["python_version"],
                            suite["providers"],
                            suite.get("gpu", False),
                        ]
                    ),
                ],
                cwd=ROOT,
                check=True,
            )
            subprocess.run(
                [str(python), "-m", "pytest", "-p", "required_suite", *suite["tests"]],
                cwd=ROOT,
                env=env,
                check=True,
            )
            if bazel:
                targets = suite.get("bazel")
                if not targets:
                    raise ValueError(f"Suite {selected} has no independent Bazel suite")
                subprocess.run(
                    ["bazel", "test", "--lockfile_mode=error", *targets],
                    cwd=ROOT,
                    check=True,
                )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="operation", required=True)
    matrix = sub.add_parser("matrix")
    matrix.add_argument("--write", action="store_true")
    matrix.add_argument("--check", action="store_true")
    setup = sub.add_parser("setup", help="Check host prerequisites; optionally uv sync")
    setup.add_argument("--profile", choices=metadata()["profiles"], required=True)
    setup.add_argument("--sync", action="store_true")
    run = sub.add_parser("run")
    run.add_argument("--profile", choices=metadata()["profiles"], required=True)
    run.add_argument(
        "--suite", choices=metadata()["suites"], action="append", required=True
    )
    run.add_argument("--bazel", action="store_true")
    artifact = sub.add_parser(
        "artifact", help="Produce a transferable verified wheel bundle"
    )
    artifact.add_argument("--profile", choices=("ubuntu", "macos"), required=True)
    artifact.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.operation == "matrix":
        content = support_table()
        path = ROOT / "third_party/support.md"
        if args.write:
            path.write_text(content)
        elif args.check:
            if path.read_text() != content:
                raise ValueError("Stale support table; run matrix --write")
        else:
            print(content, end="")
    elif args.operation == "setup":
        record = profile(args.profile)
        check_host(record)
        check_pins()
        print(f"Host prerequisites available for {args.profile}")
        if args.sync and args.profile == "ros":
            raise ValueError("Build the ROS profile with run --profile ros --suite ros")
        if args.sync:
            subprocess.run(sync_command(record), cwd=ROOT, check=True)
    elif args.operation == "artifact":
        from tools.validation.artifact import create

        create(args.profile, args.output)
        subprocess.run(["python3", str(args.output / "verify_bundle.py")], check=True)
    else:
        run_suites(args.profile, args.suite, bazel=args.bazel)


if __name__ == "__main__":
    main()
