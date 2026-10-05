"""Assemble and evaluate the camera-protection effort filter without simulation."""

from argparse import ArgumentParser
from collections.abc import Sequence
from dataclasses import dataclass

import numpy as np

from robo_arch.core.config.loading import load_run
from robo_arch.core.controllers.cbf.drake import build_filter
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.devices import load_definitions
from robo_arch.core.worlds.drake.scene import build_controller_model
from robo_arch.scenarios.camera_protection.setup import prepare

DEFAULT_RUN = "package://robo_arch/scenarios/camera_protection/scenario.yaml"


@dataclass(frozen=True)
class Summary:
    """Values worth inspecting after one nominal command is filtered."""

    geometry_constraints: int
    velocity_constraints: int
    minimum_clearance_m: float
    effort_correction_norm_Nm: float


def evaluate(reference: str) -> Summary:
    """Resolve declarations, build nominal dynamics, and filter zero effort."""
    run = load_run(reference)
    devices = resolve_devices(run.scene)
    definitions = load_definitions(run.scene, run.world)
    setup = prepare(run, devices, definitions)

    # Controller dynamics are independent of the simulated plant. The same model
    # can protect state supplied by another supported world.
    model = build_controller_model(
        setup.robot,
        setup.definition,
        sensors=devices.sensors,
        definitions=definitions,
    )
    safety = build_filter(
        model=model,
        joints=setup.definition.joints,
        geometry=setup.geometry,
        parameters=setup.control,
    )
    positions = (
        setup.definition.default_positions
        if setup.robot.initial_positions is None
        else setup.robot.initial_positions
    )
    state = np.r_[positions, np.zeros(len(positions))]
    safety.validate_initial_state(state)

    result = safety.filter(state, np.zeros(len(positions)))
    return Summary(
        geometry_constraints=safety.constraint_count,
        velocity_constraints=len(safety.velocity_bound_names),
        minimum_clearance_m=float(np.min(safety.clearances(state))),
        effort_correction_norm_Nm=float(np.linalg.norm(result.effort)),
    )


def main(argv: Sequence[str] | None = None) -> int:
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        default=DEFAULT_RUN,
        help="Camera-protection YAML path or package://robo_arch/... resource",
    )
    args = parser.parse_args(argv)
    summary = evaluate(args.run)
    print(f"geometry_constraints: {summary.geometry_constraints}")
    print(f"velocity_constraints: {summary.velocity_constraints}")
    print(f"minimum_clearance_m: {summary.minimum_clearance_m:.6f}")
    print(f"effort_correction_norm_Nm: {summary.effort_correction_norm_Nm:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
