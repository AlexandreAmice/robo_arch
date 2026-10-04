"""Run or inspect the camera-protection demonstration in Drake."""

import argparse
import hashlib
import json
import shlex
import sys
from dataclasses import replace
from functools import partial
from importlib.metadata import version
from importlib.resources import files
from pathlib import Path
from typing import Any

import numpy as np
from pydantic import TypeAdapter

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_run, resolve_resource
from robo_arch.scenarios.camera_protection.configuration import parameters_for


def default_run() -> str:
    return "package://robo_arch/scenarios/camera_protection/scenario.yaml"


def application_hashes() -> dict[str, str]:
    """Identify source and physical assets used when replaying a saved run."""
    root = Path(str(files("robo_arch")))
    return {
        path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in sorted(root.rglob("*"))
        if path.is_file()
        and path.suffix in {".py", ".yaml", ".urdf", ".sdf", ".obj"}
        and "tests" not in path.relative_to(root).parts
    }


def load_inspection(path: Path) -> tuple[RunConfiguration, bool]:
    report = json.loads(path.read_text())
    if report["application_sha256"] != application_hashes():
        raise ValueError(
            "Installed source/assets changed since this run; inspect its original HTML/NPZ instead"
        )
    if any(
        version(name) != recorded
        for name, recorded in report["package_versions"].items()
    ):
        raise ValueError(
            "Dependency versions changed since this run; inspect its original HTML/NPZ instead"
        )
    return TypeAdapter(RunConfiguration).validate_python(
        report["configuration"]
    ), report["filtered"]


def evaluate(
    run: RunConfiguration,
    trace: dict[str, np.ndarray],
    *,
    filtered: bool,
    description: dict[str, Any],
) -> dict[str, Any]:
    control, task = parameters_for(run)
    pair_count = len(description["pair_names"])
    channel = "cbf/diagnostics" if filtered else "baseline/clearance"
    samples = trace[channel]
    clearance = samples[:, :pair_count]
    unsafe = trace[channel + "/times"] < task.retreat_time
    correction = float(samples[unsafe, 5 * pair_count].max()) if filtered else 0.0
    error = float(np.max(np.abs(trace[control.robot + "/q"][-1] - task.retreat_target)))
    minimum = float(clearance.min())
    result = {
        "filtered": filtered,
        "minimum_clearance_m": minimum,
        "maximum_torque_correction_Nm": correction,
        "retreat_error_rad": error,
        "duration_seconds": float(trace["times"][-1]),
        "protected_instances": list(control.protected),
        "pair_count": pair_count,
        "sphere_pair_count": len(description["pairs"]),
        "plane_pair_count": len(description["plane_pairs"]),
        "success": minimum >= -1e-5 and correction > 1e-3 and error <= task.tolerance
        if filtered
        else minimum < -1e-3,
    }
    if description["plane_pairs"]:
        result["minimum_ground_clearance_m"] = float(
            clearance[:, len(description["pairs"]) :].min()
        )
    if filtered:
        result["minimum_cbf_residual"] = float(
            samples[:, 3 * pair_count : 4 * pair_count].min()
        )
        result["maximum_qp_solve_seconds"] = float(samples[:, 5 * pair_count + 1].max())
        result["success"] = bool(
            result["success"]
            and result["minimum_cbf_residual"] >= -control.residual_tolerance
            and np.all(samples[:, 5 * pair_count + 2] == 1)
        )
    return result


def run_scenario(
    run: RunConfiguration,
    *,
    filtered: bool = True,
    recording: Path | None = None,
    metadata: Path | None = None,
    keep_viewer_open: bool = False,
) -> dict[str, Any]:
    """Retain resolved inputs and partial evidence even when the filter stops."""
    if recording is not None:
        visual = run.world_config.visualization
        mode = (
            "live_and_record"
            if visual.mode in {"live", "live_and_record"}
            else "record"
        )
        run = replace(
            run,
            world_config=run.world_config.model_copy(
                update={"visualization": visual.model_copy(update={"mode": mode})}
            ),
        )
    metadata = (
        Path(metadata)
        if metadata is not None
        else (Path(recording).with_suffix(".json") if recording is not None else None)
    )
    trace_path = metadata.with_suffix(".npz") if metadata is not None else None
    previous_trace_stamp = (
        trace_path.stat().st_mtime_ns
        if trace_path is not None and trace_path.exists()
        else None
    )
    description = {}
    report = {
        "configuration": TypeAdapter(RunConfiguration).dump_python(run, mode="json"),
        "filtered": filtered,
        "status": "starting",
        "geometry": description,
    }
    if metadata is not None:
        metadata.parent.mkdir(parents=True, exist_ok=True)
        report["inspection_command"] = shlex.join(
            [
                "uv",
                "run",
                "python",
                "-m",
                "robo_arch.scenarios.camera_protection.run",
                "--inspect",
                str(metadata.resolve()),
                "--visualization",
                "live_and_record",
            ]
        )
    try:
        control, _ = parameters_for(run)
        report["application_sha256"] = application_hashes()
        report["package_versions"] = {
            name: version(name) for name in ("drake", "numpy", "pydantic")
        }
        resources = list(run.resources) + [
            resolve_resource(resource) for resource in control.profiles.values()
        ]
        report["resource_sha256"] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            if path.is_file()
            else "unavailable"
            for path in resources
        }
        from robo_arch.core.worlds.drake.scenario import run_scenario as execute
        from robo_arch.scenarios.camera_protection.drake import configure

        native = execute(
            run,
            configure=partial(
                configure, run=run, filtered=filtered, description=description
            ),
            recording=recording,
            trace_path=trace_path,
            keep_viewer_open=keep_viewer_open,
        )
        result = evaluate(
            run, native["trace"], filtered=filtered, description=description
        )
        result.update(
            simulation_wall_seconds=native["simulation_wall_seconds"],
            realtime_rate=native["realtime_rate"],
        )
        report.update(status="passed" if result["success"] else "failed", result=result)
        return result
    finally:
        failure = sys.exception()
        if failure is not None:
            report.update(status="error", error=f"{type(failure).__name__}: {failure}")
            if hasattr(failure, "snapshot"):
                report["failure_snapshot"] = failure.snapshot
        current_trace = (
            trace_path is not None
            and trace_path.exists()
            and trace_path.stat().st_mtime_ns != previous_trace_stamp
        )
        report["trace"] = str(trace_path.resolve()) if current_trace else None
        if metadata is not None:
            metadata.write_text(json.dumps(report, indent=2) + "\n")
            print(f"Inspect this run: {report['inspection_command']}", file=sys.stderr)
        if current_trace:
            from robo_arch.scenarios.camera_protection.plotting import plot_trace

            plot_trace(trace_path, filtered=filtered, description=description)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    inputs = parser.add_mutually_exclusive_group()
    inputs.add_argument("--run", default=default_run())
    inputs.add_argument("--inspect", type=Path)
    parser.add_argument(
        "--baseline", action="store_true", help="Run the unfiltered comparison"
    )
    parser.add_argument(
        "--visualization", choices=("off", "live", "record", "live_and_record")
    )
    parser.add_argument("--record", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    if args.inspect:
        run, filtered = load_inspection(args.inspect)
    else:
        run = load_run(args.run)
        filtered = not args.baseline
    visual = run.world_config.visualization
    visual = visual.model_copy(
        update={
            "mode": args.visualization or visual.mode,
            "open_browser": not args.no_browser,
        }
    )
    run = replace(
        run, world_config=run.world_config.model_copy(update={"visualization": visual})
    )
    suffix = "filtered" if filtered else "baseline"
    metadata = args.metadata or (
        args.inspect.with_name(args.inspect.stem + "_inspection.json")
        if args.inspect
        else Path(f"recordings/camera_protection_{suffix}.json")
    )
    recording = args.record
    if recording is None and visual.mode in {"record", "live_and_record"}:
        recording = metadata.with_suffix(".html")
    result = run_scenario(
        run,
        filtered=filtered,
        recording=recording,
        metadata=metadata,
        keep_viewer_open=True,
    )
    print(json.dumps(result, indent=2))
    if not result["success"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
