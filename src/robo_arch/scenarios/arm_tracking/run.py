"""Run arm tracking with explicit native world and visualization settings."""

import argparse
import hashlib
import json
import os
import shlex
import sys
import uuid
from dataclasses import asdict, replace
from importlib.metadata import distributions
from importlib.resources import files
from pathlib import Path

from pydantic import BaseModel, TypeAdapter

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_run, load_world, resolve_resource
from robo_arch.core.config.worlds import parse_world
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.drake.config import DrakeWorld
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.scenarios.arm_tracking.control import parameters_for
from robo_arch.scenarios.arm_tracking.evaluation import evaluate, tracking_tasks


def default_run() -> str:
    return "package://robo_arch/scenarios/arm_tracking/scenario.yaml"


def _json_value(value: object) -> object:
    if isinstance(value, BaseModel):
        return value.model_dump(mode="json")
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot serialize {type(value).__name__}")


def _inspection_command(run: RunConfiguration, metadata: Path) -> str:
    args = ["uv", "run", "--locked"]
    if run.world == "isaac":
        args.extend(["--project", "third_party/isaac"])
    args.extend(
        [
            "python",
            "-m",
            "robo_arch.scenarios.arm_tracking.run",
            "--inspect",
            str(metadata.resolve()),
            "--visualization",
            "live_and_record" if run.world == "drake" else "live",
        ]
    )
    return shlex.join(args)


def load_inspection(path: Path) -> RunConfiguration:
    """Restore resolved inputs, including test overrides, without reloading YAML.

    Device assets and executable code still come from the current installation.
    The report records configuration hashes and installed package versions.
    """
    report = json.loads(path.read_text(encoding="utf-8"))
    run = TypeAdapter(RunConfiguration).validate_python(report["configuration"])
    if not 0 < run.duration < float("inf"):
        raise ValueError("Inspection duration must be finite and positive")
    return run


def _prepare_report(run: RunConfiguration, destination: Path) -> dict:
    """Save the resolved world profile before execution, including on failure."""
    destination.parent.mkdir(parents=True, exist_ok=True)
    profile = destination.with_suffix(".world.json").resolve()
    profile.write_text(
        run.world_config.model_dump_json(indent=2) + "\n", encoding="utf-8"
    )
    installed = {
        dist.metadata["Name"].lower().replace("_", "-"): dist.version
        for dist in distributions()
    }
    versions = {}
    for package in (
        "robo-arch",
        "drake",
        "robo-arch-native",
        *(("isaacsim",) if run.world == "isaac" else ()),
    ):
        versions[package] = installed.get(package, "not installed")
    package_root = Path(str(files("robo_arch")))
    application_hashes = {
        path.relative_to(package_root).as_posix(): hashlib.sha256(
            path.read_bytes()
        ).hexdigest()
        for path in sorted(package_root.rglob("*"))
        if path.is_file()
        and not {"__pycache__", "tests"}.intersection(
            path.relative_to(package_root).parts
        )
        and path.name not in {"BUILD.bazel", "README.md"}
        and path.suffix not in {".pyc", ".pyo"}
    }
    return {
        "application_sha256": application_hashes,
        "run_id": str(uuid.uuid4()),
        "status": "starting",
        "configuration": asdict(run),
        "configuration_sha256": {
            str(path): (
                hashlib.sha256(path.read_bytes()).hexdigest()
                if path.is_file()
                else "unavailable"
            )
            for path in run.resources
        },
        "package_versions": versions,
        "inspection_command": _inspection_command(run, destination),
    }


def run_scenario(
    run: RunConfiguration,
    *,
    recording: Path | None = None,
    trace_path: Path | None = None,
    metadata: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict:
    """Execute the configured world; optional files retain run and failure evidence.

    Supplying a recording path explicitly requests native recording. Live viewers
    remain open after execution only when requested by the interactive caller.
    """
    if recording is not None:
        payload = run.world_config.model_dump()
        mode = payload["visualization"]["mode"]
        payload["visualization"]["mode"] = (
            "live_and_record" if mode in {"live", "live_and_record"} else "record"
        )
        run = replace(run, world_config=parse_world(payload))
    visual = run.world_config.visualization
    if visual.mode in {"record", "live_and_record"} and recording is None:
        recording = Path(f"recordings/arm_tracking_{run.world}.html")
    if recording is not None:
        recording = recording.resolve()
        recording.parent.mkdir(parents=True, exist_ok=True)
    if metadata is None and recording is not None:
        metadata = recording.with_suffix(".json")
    if trace_path is None and metadata is not None:
        trace_path = metadata.with_suffix(".npz")
    report = _prepare_report(run, metadata) if metadata is not None else None
    try:
        devices = resolve_devices(run.scene)
        definitions = load_definitions(run.scene, run.world)
        names = [robot.name for robot in devices.robots]
        parameters = parameters_for(run, names)
        if run.task.type != "joint_tracking":
            raise ValueError(f"Unsupported task evaluator: {run.task.type}")
        task = tracking_tasks(run.task.parameters)
        if set(task) != set(names):
            raise ValueError("Task must select every configured robot exactly once")
        wrench_names = set(run.task.parameters.get("wrenches", {}))
        available = {
            sensor.name
            for sensor in devices.sensors
            if definitions.sensors[sensor.model].kind == "wrench"
        }
        if wrench_names and (not run.sensors_enabled or not wrench_names <= available):
            raise ValueError("Wrench checks require selected, enabled wrench sensors")
        if isinstance(run.world_config, DrakeWorld):
            result = _run_drake(
                run,
                parameters,
                task,
                recording,
                keep_viewer_open,
                trace_path,
            )
        elif isinstance(run.world_config, IsaacWorld):
            result = _run_isaac(
                run,
                parameters,
                task,
                trace_path,
                keep_viewer_open,
            )
        else:
            raise ValueError("No hardware execution runner for arm tracking")
        result["world_configuration"] = run.world_config.model_dump(mode="json")
        if report is not None:
            report.update(
                status="passed" if result["success"] else "failed", result=result
            )
        return result
    finally:
        if report is not None:
            failure = sys.exception()
            if failure is not None:
                report.update(
                    status="error", error=f"{type(failure).__name__}: {failure}"
                )
            metadata.write_text(
                json.dumps(report, default=_json_value, indent=2) + "\n",
                encoding="utf-8",
            )
            print(f"Run metadata: {metadata.resolve().as_uri()}", file=sys.stderr)
            print(f"Inspect this run: {report['inspection_command']}", file=sys.stderr)


def _results(run, tasks, trace):
    import numpy as np

    wrench_checks = {}
    for name, check in run.task.parameters.get("wrenches", {}).items():
        samples = np.asarray(trace.values[name + "/wrench"])[1:, :3]
        peak = float(np.max(np.linalg.norm(samples, axis=1))) if len(samples) else 0.0
        wrench_checks[name] = {
            "peak_force_N": peak,
            "success": peak >= check["min_peak_force_N"],
        }
    robots = {
        name: {
            "robot": name,
            "initial_positions_rad": trace.values[name + "/q"][0].tolist(),
            "final_positions_rad": trace.values[name + "/q"][-1].tolist(),
            **evaluate(task, tuple(trace.values[name + "/q"][-1])),
        }
        for name, task in tasks.items()
    }
    result = {
        "world": run.world,
        "controller": run.autonomy.controller,
        "duration_seconds": trace.times[-1],
        "robots": robots,
        "success": all(
            value["success"] for value in (*robots.values(), *wrench_checks.values())
        ),
        "wrench_checks": wrench_checks,
        "final_wrenches_N_Nm": {
            name.removesuffix("/wrench"): values[-1].tolist()
            for name, values in trace.values.items()
            if name.endswith("/wrench")
        },
    }
    if len(robots) == 1:
        single = next(iter(robots.values()))
        result.update({key: value for key, value in single.items() if key != "success"})
    return result


def _run_drake(run, parameters, tasks, recording, keep_viewer_open, trace_path):
    from functools import partial

    from robo_arch.core.worlds.drake.scenario import run_scenario as execute
    from robo_arch.scenarios.arm_tracking.drake import configure

    result = execute(
        run,
        configure=partial(
            configure,
            run=run,
            parameters=parameters,
            desired_positions={name: task.target for name, task in tasks.items()},
        ),
        recording=recording,
        trace_path=trace_path,
        keep_viewer_open=keep_viewer_open,
    )
    return {**_results(run, tasks, result.pop("trace")), **result}


def _run_isaac(run, parameters, tasks, trace_path, keep_viewer_open):
    from functools import partial

    from robo_arch.core.worlds.isaac.scenario import run_scenario as execute
    from robo_arch.scenarios.arm_tracking.isaac import configure

    result = execute(
        run,
        configure=partial(configure, run=run, parameters=parameters, tasks=tasks),
        trace_path=trace_path,
        keep_viewer_open=keep_viewer_open,
    )
    return {**_results(run, tasks, result.pop("trace")), **result}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument(
        "--run", default=default_run(), help="Scenario file or package URI"
    )
    inputs.add_argument(
        "--inspect", type=Path, help="Restore resolved inputs from run metadata"
    )
    selection = parser.add_mutually_exclusive_group()
    selection.add_argument(
        "--world",
        choices=("drake", "isaac", "real"),
        help="Replace the world configuration with native defaults",
    )
    selection.add_argument(
        "--world-config", help="Complete world configuration file or package URI"
    )
    viewing = parser.add_mutually_exclusive_group()
    viewing.add_argument(
        "--visualization", choices=("off", "live", "record", "live_and_record")
    )
    viewing.add_argument(
        "--headless", action="store_true", help="Disable the viewer; preserve sensors"
    )
    parser.add_argument(
        "--record", type=Path, help="Native recording output (Drake HTML)"
    )
    parser.add_argument(
        "--trace", type=Path, help="Measured states, efforts and wrench NPZ output"
    )
    parser.add_argument(
        "--metadata", type=Path, help="Effective run configuration and result JSON"
    )
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument(
        "--no-sensors",
        action="store_true",
        help="Disable observations; keep sensor bodies, mass and collisions",
    )
    args = parser.parse_args()
    run = load_inspection(args.inspect) if args.inspect else load_run(args.run)
    world = run.world_config
    if args.world is not None:
        world = parse_world({"type": args.world})
        run = replace(run, world_source=None)
    elif args.world_config is not None:
        world = load_world(args.world_config)
        source = (
            resolve_resource(args.world_config)
            if args.world_config.startswith("package:")
            else Path(args.world_config).resolve()
        )
        run = replace(run, world_source=source)
    payload = world.model_dump()
    if args.visualization is not None:
        payload["visualization"]["mode"] = args.visualization
    if args.headless:
        payload["visualization"]["mode"] = "record" if args.record else "off"
    if args.record is not None:
        if args.visualization == "off":
            parser.error("--record conflicts with --visualization off")
        mode = payload["visualization"]["mode"]
        payload["visualization"]["mode"] = (
            "live_and_record" if mode in {"live", "live_and_record"} else "record"
        )
    if args.no_browser and isinstance(world, DrakeWorld):
        payload["visualization"]["open_browser"] = False
    # Validate overrides as a complete native config, including unsupported modes.
    run = replace(run, world_config=parse_world(payload))
    if args.no_sensors:
        run = replace(run, sensors_enabled=False)
    destination = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", "recordings"))
    metadata = args.metadata or (
        args.record.with_suffix(".json")
        if args.record
        else (
            args.inspect.with_name(args.inspect.stem + "_inspection.json")
            if args.inspect
            else destination / f"arm_tracking_{run.world}.json"
        )
    )
    recording = args.record
    if (
        args.inspect
        and recording is None
        and run.world_config.visualization.mode in {"record", "live_and_record"}
    ):
        recording = metadata.with_suffix(".html")
    result = run_scenario(
        run,
        recording=recording,
        trace_path=args.trace,
        metadata=metadata,
        keep_viewer_open=True,
    )
    trace = args.trace or metadata.with_suffix(".npz")
    if trace.exists():
        from robo_arch.core.worlds.traces import plot_trace

        print(
            f"Measured trace plot: {plot_trace(trace).resolve().as_uri()}",
            file=sys.stderr,
        )
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
