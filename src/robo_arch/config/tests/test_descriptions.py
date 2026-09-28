"""Representative I0 records; no implicit resolution or world startup."""

from dataclasses import replace
from pathlib import Path

import pytest
from pydantic import Field

from robo_arch.config.descriptions import (
    CalibrationReference,
    ChildPort,
    CompositionDescription,
    Connection,
    ExposedInput,
    ExposedOutput,
    FileReference,
    InstanceDescription,
    ScenarioDescription,
)
from robo_arch.config.resolved import (
    ResolvedComponent,
    ResolvedComposition,
    ResolvedConfiguration,
    ResolvedInput,
    ResolvedOutput,
    ResolvedPort,
    ResourceVersion,
)
from robo_arch.config.world import (
    CommandBinding,
    ConstructionContext,
    MeasurementBinding,
    ScenarioRealization,
    WorldPort,
)
from robo_arch.contracts.components import (
    ComponentDeclaration,
    ExecutionKind,
    FactoryReference,
    ImplementationCapabilities,
    ImplementationRegistration,
    Parameters,
    ResetRequirements,
    TimingRequirements,
    Transfer,
)
from robo_arch.contracts.ports import (
    ArrayOwnership,
    CommandMode,
    PortDeclaration,
    PortDescription,
    TimestampConvention,
)


class Gains(Parameters):
    kp: float = Field(default=10.0, gt=0)


@pytest.fixture
def controller():
    observation = PortDescription(
        quantity="joint_position",
        units="rad",
        dimensions=("robot.num_joints",),
        timestamp=TimestampConvention(clock="world", event="measurement_capture"),
        ownership=ArrayOwnership.BORROWED_READ_ONLY,
        robot="ur7e",
    )
    command = PortDescription(
        quantity="joint_position_command",
        units="rad",
        dimensions=("robot.num_joints",),
        timestamp=TimestampConvention(clock="world", event="command_application"),
        ownership=ArrayOwnership.OWNED,
        robot="ur7e",
        command_mode=CommandMode.POSITION,
    )
    return ComponentDeclaration(
        identifier="position_tracking",
        parameter_schema=Gains,
        inputs=(PortDeclaration(name="measured", description=observation),),
        outputs=(PortDeclaration(name="command", description=command),),
        implementations=(
            ImplementationRegistration(
                world="isaac",
                variant="default",
                factory=FactoryReference(
                    module="example_components.position_tracking.isaac",
                    attribute="build",
                ),
                capabilities=ImplementationCapabilities(
                    execution=ExecutionKind.SCALAR,
                    devices=("cpu",),
                    transfers=(
                        Transfer(
                            source_device="cuda",
                            destination_device="cpu",
                            quantity="joint_position",
                            synchronizes=True,
                        ),
                        Transfer(
                            source_device="cpu",
                            destination_device="cuda",
                            quantity="joint_position_command",
                            synchronizes=False,
                        ),
                    ),
                    timing=TimingRequirements(
                        trigger="sampled",
                        period_seconds=0.002,
                        clock="world",
                        immediate_inputs=("measured",),
                    ),
                    reset=ResetRequirements(
                        initialization_required=True,
                        reset_supported=True,
                        selective_reset_supported=False,
                    ),
                ),
            ),
        ),
    )


@pytest.fixture
def composition(controller):
    return CompositionDescription(
        identifier="tracking_pair",
        parameter_schema=Parameters,
        children=(
            InstanceDescription(name="left", definition=controller),
            InstanceDescription(name="right", definition=controller),
        ),
        connections=(),
        inputs=(
            ExposedInput(
                name="measured",
                destinations=(
                    ChildPort(child="left", port="measured"),
                    ChildPort(child="right", port="measured"),
                ),
            ),
        ),
        outputs=(
            ExposedOutput(
                name="command", source=ChildPort(child="left", port="command")
            ),
        ),
    )


@pytest.fixture
def scenario():
    def reference(path):
        return FileReference(path=path, declared_in=Path("configs/scenarios/arm.yaml"))

    return ScenarioDescription(
        robot=reference("../robots/ur7e.yaml"),
        sensors=reference("../sensors/none.yaml"),
        objects=reference("objects.yaml"),
        task=reference("tracking.yaml"),
        layout=reference("bench.yaml"),
        calibration=(
            CalibrationReference(
                data=reference("../installations/lab/calibration.yaml"),
                installation="lab",
                device_ids=("robot-serial",),
                mounting_arrangement="bench-mount-v2",
            ),
        ),
    )


def test_nested_composition_preserves_input_fanout(composition):
    nested = CompositionDescription(
        identifier="outer",
        parameter_schema=Parameters,
        children=(InstanceDescription(name="inner", definition=composition),),
        connections=(),
        inputs=(
            ExposedInput(
                name="measured",
                destinations=(ChildPort(child="inner", port="measured"),),
            ),
        ),
        outputs=(
            ExposedOutput(
                name="command", source=ChildPort(child="inner", port="command")
            ),
        ),
    )
    assert nested.children[0].definition is composition
    assert len(composition.inputs[0].destinations) == 2
    assert not hasattr(nested.inputs[0], "description")  # Types derive in S0.
    connection = Connection(
        source=ChildPort(child="reference", port="desired"),
        destination=ChildPort(child="controller", port="desired"),
    )
    assert connection.source.child != connection.destination.child


def test_monolithic_policy_needs_no_mandatory_component_slots(controller):
    policy = ComponentDeclaration(
        identifier="joint_policy",
        parameter_schema=Parameters,
        inputs=controller.inputs,
        outputs=controller.outputs,
        implementations=(),  # Missing support is represented; S0 must reject it.
    )
    autonomy = InstanceDescription(name="policy", definition=policy)
    assert autonomy.definition.identifier == "joint_policy"
    assert not hasattr(autonomy.definition, "children")


def test_isaac_cpu_registration_is_explicit(controller):
    implementation = controller.implementations[0]
    assert implementation.world == "isaac"
    assert implementation.capabilities.execution is ExecutionKind.SCALAR
    assert implementation.capabilities.devices == ("cpu",)
    assert implementation.capabilities.transfers[0].synchronizes
    assert implementation.approximation is None
    assert not implementation.capabilities.reset.selective_reset_supported


def test_scenario_keeps_reference_origins_and_calibration_identity(scenario):
    assert scenario.robot.path == "../robots/ur7e.yaml"
    assert scenario.robot.declared_in == Path("configs/scenarios/arm.yaml")
    assert scenario.calibration[0].installation == "lab"
    assert scenario.calibration[0].mounting_arrangement == "bench-mount-v2"


def test_resolved_records_supply_factory_context(controller, composition, scenario):
    measured = ResolvedPort(
        instance_path=("stack", "left"),
        name="measured",
        description=replace(controller.inputs[0].description, dimensions=(6,)),
    )
    command = ResolvedPort(
        instance_path=("stack", "left"),
        name="command",
        description=replace(controller.outputs[0].description, dimensions=(6,)),
    )
    leaf = ResolvedComponent(
        instance_path=("stack", "left"),
        declaration=controller,
        parameters=controller.parameter_schema.model_validate({"kp": 12.0}),
        implementation=controller.implementations[0],
        inputs=(measured,),
        outputs=(command,),
    )
    right_measured = replace(measured, instance_path=("stack", "right"))
    right_command = replace(command, instance_path=("stack", "right"))
    right_leaf = replace(
        leaf,
        instance_path=("stack", "right"),
        inputs=(right_measured,),
        outputs=(right_command,),
    )
    # Manual records exercise the boundary; S0 will build/check this graph.
    stack = ResolvedComposition(
        instance_path=("stack",),
        declaration=composition,
        parameters=Parameters(),
        children=(leaf, right_leaf),
        connections=(),
        inputs=(
            ResolvedInput(
                name="measured",
                description=measured.description,
                destinations=(measured, right_measured),
            ),
        ),
        outputs=(
            ResolvedOutput(
                name="command", description=command.description, source=command
            ),
        ),
    )
    configuration = ResolvedConfiguration(
        scenario=scenario,
        world="isaac",
        autonomy=stack,
        resources=(ResourceVersion(resource="ur7e", version="test-model-v1"),),
    )
    world_measurement = WorldPort(
        name="arm_position", description=measured.description, handle=object()
    )
    world_command = WorldPort(
        name="arm_position_command", description=command.description, handle=object()
    )
    context = ConstructionContext(
        configuration=configuration,
        scenario=ScenarioRealization(
            measurements=(world_measurement,),
            commands=(world_command,),
            controller_models={"ur7e": object()},
            services={},
        ),
        measurements=(
            MeasurementBinding(
                source=world_measurement, destinations=(measured, right_measured)
            ),
        ),
        commands=(CommandBinding(source=command, destination=world_command),),
    )
    assert context.configuration.autonomy.children[0].parameters.kp == 12.0
    assert context.commands[0].destination is world_command
    assert context.measurements[0].destinations == (measured, right_measured)
    assert context.configuration.resources[0].version == "test-model-v1"


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
