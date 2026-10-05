"""Native Drake composition of a nominal controller and a sphere CBF filter."""

from typing import Any

import numpy as np
from pydrake.systems.framework import (
    BasicVector,
    Context,
    DiagramBuilder,
    LeafSystem,
    OutputPort,
)

from robo_arch.core.config.declarations import RunConfiguration
from robo_arch.core.controllers.cbf.drake import (
    CbfClearanceSystem,
    SphereCbfSystem,
    add_ground_illustrations,
    add_sphere_illustrations,
    build_filter,
)
from robo_arch.core.controllers.selection import scalar_system, select_controller
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.drake.scene import DrakeScene
from robo_arch.scenarios.camera_protection.configuration import (
    TaskParameters,
)
from robo_arch.scenarios.camera_protection.reference import desired_state
from robo_arch.scenarios.camera_protection.setup import prepare


class Reference(LeafSystem):
    """Attempt an obstructed target, then return to a feasible retreat target."""

    def __init__(self, task: TaskParameters, initial: tuple[float, ...]):
        super().__init__()
        self.task = task
        self.initial = np.asarray(initial)
        self.DeclareVectorOutputPort(
            "desired_state", BasicVector(2 * len(task.unsafe_target)), self.output
        )

    def output(self, context: Context, output: BasicVector) -> None:
        q, v = desired_state(
            np.asarray(context.get_time()),
            self.initial,
            np.asarray(self.task.unsafe_target),
            np.asarray(self.task.retreat_target),
            retreat_time=self.task.retreat_time,
            transition_seconds=self.task.transition_seconds,
        )
        output.SetFromVector(np.r_[q, v])


def configure(
    builder: DiagramBuilder,
    scene: DrakeScene,
    *,
    run: RunConfiguration,
    filtered: bool = True,
    description: dict[str, Any] | None = None,
) -> dict[str, OutputPort]:
    setup = prepare(run, resolve_devices(run.scene), scene.definitions)
    control, task = setup.control, setup.task
    robot = control.robot
    if set(scene.robots) != {robot}:
        raise ValueError(
            "camera_protection requires the selected single controlled arm"
        )
    model = scene.controller_models[robot]
    joints = setup.definition.joints
    setup.validate_targets(
        model.GetPositionLowerLimits(), model.GetPositionUpperLimits()
    )
    geometry = setup.geometry
    cbf = build_filter(
        model=model,
        geometry=geometry,
        joints=joints,
        parameters=control,
    )
    if description is not None:
        description.update(
            setup.describe(
                pair_names=cbf.pair_names,
                velocity_bound_names=cbf.velocity_bound_names,
                qp_constraint_count=cbf.qp_constraint_count,
            )
        )
    cbf.validate_initial_state(
        np.r_[scene.initial_positions[robot], np.zeros(len(joints))]
    )
    selection = select_controller(run.world_config, "cbf")
    if description is not None:
        description["controller_selection"] = selection.describe()
    nominal = builder.AddSystem(
        scalar_system(
            control.nominal_controller,
            model=model,
            parameters=control.nominal,
            joints=joints,
        )
    )
    nominal.set_name("nominal_" + control.nominal_controller)
    reference = builder.AddSystem(Reference(task, scene.initial_positions[robot]))
    state = scene.plant.get_state_output_port(scene.robots[robot])
    builder.Connect(state, nominal.GetInputPort("estimated_state"))
    builder.Connect(reference.get_output_port(), nominal.GetInputPort("desired_state"))
    nominal_effort = nominal.get_output_port(0)
    effort = nominal_effort
    diagnostics = {}
    if filtered:
        safety = builder.AddSystem(SphereCbfSystem(filter=cbf))
        safety.set_name("camera_protection_cbf")
        builder.Connect(state, safety.GetInputPort("estimated_state"))
        builder.Connect(effort, safety.GetInputPort("nominal_effort"))
        effort = safety.GetOutputPort("effort")
        diagnostics["cbf/diagnostics"] = safety.GetOutputPort("diagnostics")
    else:
        observer = builder.AddSystem(CbfClearanceSystem(cbf))
        builder.Connect(state, observer.state)
        diagnostics["baseline/clearance"] = observer.get_output_port()
    builder.Connect(effort, scene.plant.get_actuation_input_port(scene.robots[robot]))
    diagnostics["nominal_effort"] = nominal_effort
    diagnostics["commanded_effort"] = effort
    add_sphere_illustrations(scene, geometry.spheres, control.protected)
    if geometry.plane_pairs:
        add_ground_illustrations(scene, control.margin)
    return diagnostics
