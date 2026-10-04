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
from robo_arch.core.controllers.cbf.assembly import resolve_geometry
from robo_arch.core.controllers.cbf.drake import (
    CbfClearanceSystem,
    SphereCbfSystem,
    build_filter,
)
from robo_arch.core.controllers.cbf.visualization import (
    add_ground_illustrations,
    add_sphere_illustrations,
)
from robo_arch.core.controllers.joint_tracking.drake import build
from robo_arch.core.worlds.assembly import resolve_devices
from robo_arch.core.worlds.drake.scene import DrakeScene
from robo_arch.scenarios.camera_protection.configuration import (
    TaskParameters,
    parameters_for,
)


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
        time = context.get_time()
        if time < self.task.retreat_time:
            start, end, elapsed = (
                self.initial,
                np.asarray(self.task.unsafe_target),
                time,
            )
        else:
            start, end = (
                np.asarray(self.task.unsafe_target),
                np.asarray(self.task.retreat_target),
            )
            elapsed = time - self.task.retreat_time
        u = float(np.clip(elapsed / self.task.transition_seconds, 0, 1))
        blend = 10 * u**3 - 15 * u**4 + 6 * u**5
        speed = (30 * u**2 - 60 * u**3 + 30 * u**4) / self.task.transition_seconds
        output.SetFromVector(
            np.r_[start + blend * (end - start), speed * (end - start)]
        )


def configure(
    builder: DiagramBuilder,
    scene: DrakeScene,
    *,
    run: RunConfiguration,
    filtered: bool = True,
    description: dict[str, Any] | None = None,
) -> dict[str, OutputPort]:
    control, task = parameters_for(run)
    robot = control.robot
    if set(scene.robots) != {robot}:
        raise ValueError(
            "camera_protection requires the selected single controlled arm"
        )
    model = scene.controller_models[robot]
    robot_instance = next(
        item for item in resolve_devices(run.scene).robots if item.name == robot
    )
    joints = scene.definitions.robots[robot_instance.model].joints
    for target in (task.unsafe_target, task.retreat_target):
        if len(target) != model.num_positions() or not np.all(np.isfinite(target)):
            raise ValueError("Targets must match the selected robot's joint order")
        if np.any(target < model.GetPositionLowerLimits()) or np.any(
            target > model.GetPositionUpperLimits()
        ):
            raise ValueError("Target exceeds robot joint limits")
    geometry = resolve_geometry(run.scene, control, ground=run.world_config.ground)
    cbf = build_filter(
        model=model,
        geometry=geometry,
        joints=joints,
        parameters=control,
    )
    if description is not None:
        description.update(
            pair_names=cbf.pair_names,
            spheres=[vars(sphere) for sphere in geometry.spheres],
            pairs=[vars(pair) for pair in geometry.pairs],
            planes=[vars(plane) for plane in geometry.planes],
            plane_pairs=[vars(pair) for pair in geometry.plane_pairs],
        )
    cbf.validate_initial_state(
        np.r_[scene.initial_positions[robot], np.zeros(len(joints))]
    )
    if control.nominal_controller == "joint_pd":
        from robo_arch.core.controllers.joint_pd.definition import JointPdParameters
        from robo_arch.core.controllers.joint_pd.drake import JointPdSystem

        nominal = builder.AddSystem(
            JointPdSystem(
                model=model,
                parameters=JointPdParameters.model_validate(
                    control.nominal.model_dump()
                ),
                joints=joints,
            )
        )
    else:
        nominal = builder.AddSystem(
            build(model=model, parameters=control.nominal, joints=joints)
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
