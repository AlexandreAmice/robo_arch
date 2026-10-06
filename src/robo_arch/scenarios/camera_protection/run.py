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

from robo_arch.core.config.cli import add_overrides, apply_overrides
from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.config.loading import load_run, resolve_resource
from robo_arch.core.worlds.launch import inspection_invocation, prepare
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
    if samples.ndim == 3:
        results = [
            evaluate(
                run,
                {
                    name: values[:, index] if values.ndim == 3 else values
                    for name, values in trace.items()
                },
                filtered=filtered,
                description=description,
            )
            for index in range(samples.shape[1])
        ]
        result = dict(results[0])
        result.update(
            batch_size=len(results),
            per_environment=results,
            success=all(item["success"] for item in results),
        )
        for name in (
            "minimum_clearance_m",
            "minimum_ground_clearance_m",
            "minimum_cbf_residual",
        ):
            if name in result:
                result[name] = min(item[name] for item in results)
        for name in ("maximum_torque_correction_Nm", "retreat_error_rad"):
            result[name] = max(item[name] for item in results)
        return result
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
        solve_times = samples[:, 5 * pair_count + 1]
        if np.isfinite(solve_times).all():
            result["maximum_qp_solve_seconds"] = float(solve_times.max())
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
        invocation = inspection_invocation(__file__)
        mode = "live_and_record" if run.world == "drake" else "live"
        report["inspection_command"] = shlex.join(
            [
                *invocation,
                "--inspect",
                str(metadata.resolve()),
                "--visualization",
                mode,
            ]
        )
    try:
        control, _ = parameters_for(run)
        report["application_sha256"] = application_hashes()
        packages = ("drake", "numpy", "pydantic")
        if run.world == "isaac":
            packages += ("torch", "moreau", "moreau-cuda13", "isaacsim", "isaaclab")
        report["package_versions"] = {name: version(name) for name in packages}
        resources = list(run.resources) + [
            resolve_resource(resource) for resource in control.profiles.values()
        ]
        report["resource_sha256"] = {
            str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            if path.is_file()
            else "unavailable"
            for path in resources
        }
        if run.world == "isaac":
            from robo_arch.core.worlds.isaac.scenario import run_scenario as execute
            from robo_arch.scenarios.camera_protection.isaac import rollout

            wiring = {
                "rollout": partial(
                    rollout,
                    run=run,
                    filtered=filtered,
                    description=description,
                    trace_path=trace_path,
                )
            }
        else:
            from robo_arch.core.worlds.drake.scenario import run_scenario as execute
            from robo_arch.scenarios.camera_protection.drake import configure

            wiring = {
                "configure": partial(
                    configure, run=run, filtered=filtered, description=description
                )
            }

        native = execute(
            run,
            **wiring,
            recording=recording,
            trace_path=trace_path,
            keep_viewer_open=keep_viewer_open,
        )
        result = evaluate(
            run, native["trace"], filtered=filtered, description=description
        )
        if run.world == "isaac":
            statistics = native["controller_statistics"][control.robot]
            items = result.get("per_environment", [result])
            for index, item in enumerate(items):
                item.update(
                    {name: float(values[index]) for name, values in statistics.items()}
                )
                item["success"] = (
                    item["minimum_clearance_m"] >= -1e-5
                    and item["maximum_torque_correction_Nm"] > 1e-3
                    and item["retreat_error_rad"] <= parameters_for(run)[1].tolerance
                    and item["minimum_cbf_residual"] >= -control.residual_tolerance
                    and item.get("minimum_joint_velocity_slack", 0) >= -1e-5
                    and item.get("minimum_velocity_cbf_residual", 0)
                    >= -control.residual_tolerance
                    if filtered
                    else item["minimum_clearance_m"] < -1e-3
                )
            for name in statistics:
                result[name] = float(
                    np.max(statistics[name])
                    if name.startswith("maximum")
                    else np.min(statistics[name])
                )
            result["success"] = all(item["success"] for item in items)
            result["clearance_evaluation_period_seconds"] = run.time_step
            result["trace_sample_period_seconds"] = (
                run.time_step * parameters_for(run)[1].log_every_n_steps
            )
            result["environment_steps_per_second"] = native[
                "environment_steps_per_second"
            ]
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
    add_overrides(parser)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--no-browser", action="store_true")
    parser.add_argument("--batch-size", type=int, help="Isaac tensor environments")
    args = parser.parse_args()
    if args.inspect:
        report = json.loads(args.inspect.read_text())
        run = TypeAdapter(RunConfiguration).validate_python(report["configuration"])
        filtered = report["filtered"]
    else:
        run = load_run(args.run)
        filtered = not args.baseline
    run = apply_overrides(run, args)
    if args.batch_size is not None:
        if run.world != "isaac":
            parser.error("--batch-size requires Isaac")
        world = type(run.world_config).model_validate(
            {**run.world_config.model_dump(), "num_envs": args.batch_size}
        )
        run = replace(run, world_config=world)
    visual = run.world_config.visualization
    updates = {"mode": visual.mode}
    if run.world == "drake":
        updates["open_browser"] = not args.no_browser
    visual = type(visual).model_validate({**visual.model_dump(), **updates})
    run = replace(
        run, world_config=run.world_config.model_copy(update={"visualization": visual})
    )
    parameters_for(run)
    prepare(run.world, live=visual.mode in {"live", "live_and_record"})
    if args.inspect:
        load_inspection(args.inspect)
    suffix = ("filtered" if filtered else "baseline") + (
        "_gpu" if run.world == "isaac" else ""
    )
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
