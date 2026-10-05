"""Opt-in native PhysX comparisons; each Kit instance owns a fresh subprocess.

Run with ROBO_ARCH_NATIVE_ISAAC=1 in third_party/isaac's cbf-gpu/test profile.
JSON, NPZ, plots and native logs retain the actual inputs and measured outcomes.
"""

import argparse
import json
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

CASES = (
    "obstacles_filtered",
    "obstacles_baseline",
    "ground_filtered",
    "ground_baseline",
)


def configured_run(case: str):
    from robo_arch.core.config.loading import load_run, load_world
    from robo_arch.scenarios.camera_protection.run import default_run
    from robo_arch.scenarios.camera_protection.tests.test_ground import (
        ground_approach_run,
    )

    run = (
        ground_approach_run() if case.startswith("ground_") else load_run(default_run())
    )
    world = load_world("package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml")
    return replace(
        run,
        world_config=world.model_copy(update={"num_envs": 2}),
        autonomy=run.autonomy.model_copy(
            update={
                "parameters": dict(
                    run.autonomy.parameters,
                )
            }
        ),
    )


@pytest.mark.skipif(
    os.environ.get("ROBO_ARCH_NATIVE_ISAAC") != "1",
    reason="Enable ROBO_ARCH_NATIVE_ISAAC=1 in the Isaac vendor profile",
)
@pytest.mark.parametrize("case", CASES)
def test_native_gpu_camera_protection(tmp_path: Path, case: str):
    destination = Path(os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR", str(tmp_path)))
    destination.mkdir(parents=True, exist_ok=True)
    metadata = destination / f"camera_gpu_{case}.json"
    log = metadata.with_suffix(".log")
    environment = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES")
    try:
        with log.open("w") as output:
            child = subprocess.run(
                [
                    sys.executable,
                    __file__,
                    "--case",
                    case,
                    "--metadata",
                    str(metadata),
                ],
                env=environment,
                stdout=output,
                stderr=subprocess.STDOUT,
                timeout=1200,
                check=False,
            )
        assert child.returncode == 0, f"Native run exited {child.returncode}; see {log}"
        report = json.loads(metadata.read_text())
        result = report["result"]
        assert report["status"] == "passed" and result["success"], result
        assert result["batch_size"] == 2
        assert result["clearance_evaluation_period_seconds"] == pytest.approx(0.001)
        filtered = case.endswith("_filtered")
        for item in result["per_environment"]:
            assert item["success"] and item["plane_pair_count"] == 3, item
            if filtered:
                assert item["minimum_clearance_m"] >= -1e-5
                assert item["minimum_cbf_residual"] >= -1e-6
                assert item["minimum_joint_velocity_slack"] >= -1e-5
                assert item["minimum_velocity_cbf_residual"] >= -1e-6
                assert item["maximum_torque_correction_Nm"] > 1e-3
                assert item["retreat_error_rad"] <= 0.03
            else:
                assert item["minimum_clearance_m"] < -1e-3
            if case.startswith("ground_"):
                assert item["minimum_ground_clearance_m"] < (
                    0.001 if filtered else -0.001
                )
            else:
                assert item["minimum_ground_clearance_m"] > 0
        assert metadata.with_suffix(".png").stat().st_size > 0
        with np.load(metadata.with_suffix(".npz")) as trace:
            assert trace["arm/q"].shape[1:] == (2, 6)
            assert trace["times"][-1] == pytest.approx(
                4.0 if case.startswith("ground_") else 6.0
            )
    finally:
        print(f"Actual trace plot: {metadata.with_suffix('.png')}; native log: {log}")
        if metadata.exists():
            report = json.loads(metadata.read_text())
            print(f"Inspect the same inputs in Isaac: {report['inspection_command']}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", choices=CASES, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    args = parser.parse_args()
    from robo_arch.scenarios.camera_protection.run import run_scenario

    run_scenario(
        configured_run(args.case),
        filtered=args.case.endswith("_filtered"),
        metadata=args.metadata,
    )


if __name__ == "__main__":
    main()
