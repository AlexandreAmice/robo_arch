"""Scenario-supported controller selections and per-robot parameters."""

from robo_arch.core.controllers.joint_pd.definition import JointPdParameters
from robo_arch.core.controllers.joint_tracking.definition import JointTrackingParameters


def parameters_for(run, names):
    """Physical composition remains independent of autonomy selection."""
    schemas = {"joint_tracking": JointTrackingParameters, "joint_pd": JointPdParameters}
    if run.autonomy.controller not in schemas:
        raise ValueError(
            f"Unsupported arm-tracking controller: {run.autonomy.controller}"
        )
    schema = schemas[run.autonomy.controller]
    values = run.autonomy.parameters
    if "robots" in values:
        if set(values) != {"robots"} or set(values["robots"]) != set(names):
            raise ValueError(
                "Controller parameters must name exactly the configured robots"
            )
        return {name: schema.model_validate(values["robots"][name]) for name in names}
    if len(names) != 1:
        raise ValueError(
            "Multiple robots require explicit per-robot controller parameters"
        )
    return {names[0]: schema.model_validate(values)}


def make_policy(run, *, model, parameters, joints, desired_state):
    from robo_arch.core.controllers.selection import scalar_policy, select_controller

    select_controller(run.world_config, run.autonomy.controller)
    return scalar_policy(
        run.autonomy.controller,
        model=model,
        parameters=parameters,
        joints=joints,
        desired_state=desired_state,
    )
