"""Bounded effort filtering and cache behavior on independent analytic plants."""

from types import SimpleNamespace

import numpy as np
import pytest
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, LogVectorOutput

from robo_arch.core.controllers.cbf.definition import (
    CbfParameters,
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)
from robo_arch.core.controllers.cbf.drake import (
    CbfClearanceSystem,
    CbfFailure,
    SphereCbfFilter,
    SphereCbfSystem,
)


def slider(*, limit=100, damping=0, vertical=False, time_step=0.0, velocity_limit=10):
    plant = MultibodyPlant(time_step)
    axis = "0 0 1" if vertical else "1 0 0"
    Parser(plant).AddModelsFromString(
        f'''<robot name="robot"><link name="base"/>
        <link name="body"><inertial><mass value="1"/>
          <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
        </inertial></link><joint name="slide" type="prismatic">
          <parent link="base"/><child link="body"/><axis xyz="{axis}"/>
          <limit lower="-3" upper="3" effort="{limit}" velocity="{velocity_limit}"/>
          <dynamics damping="{damping}"/>
        </joint></robot>''',
        "urdf",
    )
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    plant.AddJointActuator("motor", plant.GetJointByName("slide"), limit)
    plant.Finalize()
    return plant


def make_filter(*, model=None, vertical=False, **model_args):
    return SphereCbfFilter(
        model=model if model is not None else slider(vertical=vertical, **model_args),
        spheres=(
            Sphere("camera", "robot/body", (0, 0, 0), 0.1),
            Sphere("obstacle", "world", (0, 0, 1) if vertical else (1, 0, 0), 0.1),
        ),
        pairs=(SpherePair("camera", "obstacle", margin=0),),
        joints=("slide",),
        parameters=CbfParameters(alpha1=5, alpha2=5),
    )


def test_passthrough_and_analytic_projection():
    controller = make_filter()
    controller.validate_initial_state(np.array([0.5, 0]))
    np.testing.assert_allclose(controller.filter([0.5, 0], [1]).effort, [1], atol=1e-8)
    result = controller.filter([0.5, 0], [20])
    # h=.21; ddot(h)=-u, so u <= 25*.21 = 5.25 N.
    np.testing.assert_allclose(result.effort, [5.25], atol=1e-7)
    np.testing.assert_allclose(result.diagnostics[:3], [0.3, 0.21, 1.05])
    assert result.diagnostics[3] >= -1e-6
    assert result.diagnostics[4] == 1
    assert result.diagnostics[-1] == 1


def test_torque_bounds_are_inside_qp():
    result = make_filter(limit=2).filter([0.5, 0], [20])
    np.testing.assert_allclose(result.effort, [2], atol=1e-7)


def test_exact_feasible_nominal_skips_solver():
    controller = make_filter()
    controller._solver = SimpleNamespace(Solve=lambda *args: pytest.fail("Unneeded QP"))
    result = controller.filter([0.5, 0], [1])
    np.testing.assert_array_equal(result.effort, [1])
    np.testing.assert_array_equal(result.diagnostics[-3:], [0, 0, 1])


def test_infeasibility_stops_with_named_snapshot():
    controller = make_filter(limit=2)
    with pytest.raises(CbfFailure, match="QP failed") as error:
        controller.filter([0.8, 1], [0], time=2.5)
    assert error.value.snapshot["time"] == 2.5
    assert "camera|obstacle" in error.value.snapshot["pairs"]
    assert error.value.snapshot["nominal_effort"] == [0.0]


@pytest.mark.parametrize("state", [[0.9, 0], [0.5, 3]])
def test_initial_domain_includes_velocity(state):
    with pytest.raises(CbfFailure, match="initial state"):
        make_filter().validate_initial_state(state)


def test_damping_contribution_is_included_exactly_once():
    controller = make_filter(damping=2)
    # At q=.5,v=.2: ddot(h)=.08+.4-u, 10hdot=-2,25h=5.25.
    np.testing.assert_allclose(
        controller.filter([0.5, 0.2], [20]).effort, [3.73], atol=1e-7
    )


def test_gravity_is_included_exactly_once():
    controller = make_filter(vertical=True)
    np.testing.assert_allclose(
        controller.filter([0.5, 0], [30]).effort, [15.06], atol=1e-7
    )


def test_independent_filter_contexts_and_callable():
    model = slider()
    first, second = make_filter(model=model), make_filter(model=model)
    command = first.wrap(lambda state, time: np.array([20]))
    expected = command(np.array([0.5, 0]), 0)
    second.filter([0.6, -0.1], [0])
    np.testing.assert_array_equal(command(np.array([0.5, 0]), 0), expected)
    assert first._state is not second._state
    assert first._program is not second._program


def test_native_cache_shared_outputs_and_context_isolation(monkeypatch):
    controller = make_filter()
    native = SphereCbfSystem(filter=controller)
    calls = []
    original = controller.filter

    def record(*args):
        calls.append(1)
        return original(*args)

    monkeypatch.setattr(controller, "filter", record)
    first = native.CreateDefaultContext()
    native.GetInputPort("estimated_state").FixValue(first, [0.5, 0])
    native.GetInputPort("nominal_effort").FixValue(first, [20])
    a = native.GetOutputPort("effort").Eval(first).copy()
    native.GetOutputPort("diagnostics").Eval(first)
    assert len(calls) == 1
    second = first.Clone()
    native.GetInputPort("estimated_state").FixValue(second, [0.6, 0])
    b = native.GetOutputPort("effort").Eval(second).copy()
    assert len(calls) == 2
    assert not np.allclose(a, b)
    np.testing.assert_array_equal(native.GetOutputPort("effort").Eval(first), a)
    assert len(calls) == 2


@pytest.mark.parametrize(
    ("state", "effort"), [([0, np.nan], [0]), ([0, 0], [np.inf]), ([0], [0])]
)
def test_invalid_inputs_fail_before_solver(state, effort):
    with pytest.raises(ValueError, match="finite"):
        make_filter().filter(state, effort)


def two_link(*, reverse_actuators=False):
    plant = MultibodyPlant(0.0)
    Parser(plant).AddModelsFromString(
        """<robot name="robot"><link name="base"/>
        <link name="first"><inertial><origin xyz="0.3 0 0"/>
          <mass value="2"/><inertia ixx="0.1" iyy="0.1" izz="0.1"
           ixy="0" ixz="0" iyz="0"/></inertial></link>
        <link name="second"><inertial><origin xyz="0.2 0 0"/>
          <mass value="1"/><inertia ixx="0.1" iyy="0.1" izz="0.1"
           ixy="0" ixz="0" iyz="0"/></inertial></link>
        <joint name="shoulder" type="revolute"><parent link="base"/>
          <child link="first"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="100" velocity="10"/>
        </joint><joint name="elbow" type="revolute"><parent link="first"/>
          <child link="second"/><origin xyz="0.8 0.1 0"/><axis xyz="0 0 1"/>
          <limit lower="-3" upper="3" effort="100" velocity="10"/>
          <dynamics damping="0.3"/>
        </joint></robot>""",
        "urdf",
    )
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("base"))
    names = ("elbow", "shoulder") if reverse_actuators else ("shoulder", "elbow")
    for name in names:
        plant.AddJointActuator(name + "_motor", plant.GetJointByName(name), 100)
    plant.Finalize()
    return plant


def two_link_filter(model, joints=("shoulder", "elbow")):
    return SphereCbfFilter(
        model=model,
        spheres=(
            Sphere("camera", "robot/first", (0.5, 0.2, 0), 0.1),
            Sphere("other_camera", "robot/second", (0.4, 0.1, 0), 0.1),
        ),
        pairs=(SpherePair("camera", "other_camera", margin=0),),
        joints=joints,
    )


def test_drake_moving_pair_second_derivative_matches_independent_difference():
    model = two_link()
    controller = two_link_filter(model)
    q, v, effort = np.array([0.4, 0.8]), np.array([0.7, -0.5]), np.array([0.3, -0.7])
    context = model.CreateDefaultContext()
    model.SetPositionsAndVelocities(context, np.concatenate((q, v)))
    forces = MultibodyForces(model)
    model.CalcForceElementsContribution(context, forces)
    acceleration = np.linalg.solve(
        model.CalcMassMatrix(context),
        model.MakeActuationMatrix() @ effort
        - model.CalcInverseDynamics(context, np.zeros(2), forces),
    )

    def h(time):
        model.SetPositions(context, q + time * v + 0.5 * time**2 * acceleration)
        centers = [
            model.CalcPointsPositions(
                context,
                model.GetFrameByName(frame),
                np.array(point),
                model.world_frame(),
            ).ravel()
            for frame, point in (("first", [0.5, 0.2, 0]), ("second", [0.4, 0.1, 0]))
        ]
        delta = centers[0] - centers[1]
        return delta @ delta - 0.2**2

    dt = 1e-4
    first = (h(dt) - h(-dt)) / (2 * dt)
    second = (h(dt) - 2 * h(0) + h(-dt)) / dt**2
    (constraint,) = controller.evaluate(np.concatenate((q, v)))
    assert constraint.coefficient @ effort + constraint.constant == pytest.approx(
        second + 10 * first + 25 * h(0), abs=2e-7
    )


def test_reordered_actuators_cannot_silently_reorder_state():
    with pytest.raises(ValueError, match="scalar q, v and actuator order"):
        two_link_filter(two_link(reverse_actuators=True), joints=("elbow", "shoulder"))


def test_grouped_frame_points_preserve_pair_order_and_relative_derivatives():
    model = two_link()
    spheres = (
        Sphere("first_tip", "robot/first", (0.5, 0.2, 0), 0.1),
        Sphere("second_tip", "robot/second", (0.4, 0.1, 0), 0.08),
        Sphere("first_side", "robot/first", (0.15, -0.2, 0), 0.09),
        Sphere("fixed", "world", (0.9, 0.1, 0.2), 0.12),
        Sphere("second_side", "robot/second", (0.2, -0.3, 0), 0.05),
    )
    pairs = (
        SpherePair("first_side", "second_side", 0.02),
        SpherePair("first_tip", "fixed", 0.03),
        SpherePair("first_tip", "second_tip", 0.01),
        SpherePair("second_side", "fixed", 0.01),
    )
    controller = SphereCbfFilter(
        model=model, spheres=spheres, pairs=pairs, joints=("shoulder", "elbow")
    )
    state, effort = np.array([0.4, 0.8, 0.7, -0.5]), np.array([0.3, -0.7])
    context = model.CreateDefaultContext()
    model.SetPositionsAndVelocities(context, state)
    forces = MultibodyForces(model)
    model.CalcForceElementsContribution(context, forces)
    acceleration = np.linalg.solve(
        model.CalcMassMatrix(context),
        effort - model.CalcInverseDynamics(context, np.zeros(2), forces),
    )
    by_name = {sphere.name: sphere for sphere in spheres}

    def distance(pair, time):
        model.SetPositions(
            context, state[:2] + time * state[2:] + 0.5 * time**2 * acceleration
        )
        points = []
        for name in (pair.first, pair.second):
            sphere = by_name[name]
            frame = (
                model.world_frame()
                if sphere.frame == "world"
                else model.GetFrameByName(sphere.frame.rsplit("/", 1)[1])
            )
            points.append(
                model.CalcPointsPositions(
                    context, frame, np.array(sphere.center), model.world_frame()
                ).ravel()
            )
        return np.linalg.norm(points[0] - points[1])

    dt = 1e-4
    rows = controller.evaluate(state)
    clearances = controller.clearances(state)
    for pair, row, clearance in zip(pairs, rows, clearances, strict=True):
        radius = by_name[pair.first].radius + by_name[pair.second].radius + pair.margin
        minus, center, plus = (distance(pair, t) ** 2 - radius**2 for t in (-dt, 0, dt))
        hdot = (plus - minus) / (2 * dt)
        hddot = (plus - 2 * center + minus) / dt**2
        assert row.h == pytest.approx(center, abs=1e-12)
        assert row.psi1 == pytest.approx(hdot + 5 * center, abs=1e-8)
        assert row.coefficient @ effort + row.constant == pytest.approx(
            hddot + 10 * hdot + 25 * center, abs=2e-7
        )
        assert row.clearance == pytest.approx(distance(pair, 0) - radius, abs=1e-12)
        assert clearance == pytest.approx(row.clearance, abs=1e-12)


def test_two_simultaneous_opposing_constraints():
    controller = SphereCbfFilter(
        model=slider(),
        spheres=(
            Sphere("camera", "robot/body", (0, 0, 0), 0.1),
            Sphere("left", "world", (0, 0, 0), 0.4),
            Sphere("right", "world", (1, 0, 0), 0.4),
        ),
        pairs=(SpherePair("camera", "left", 0), SpherePair("camera", "right", 0)),
        joints=("slide",),
    )
    controller.validate_initial_state([0.5, 0])
    result = controller.filter([0.5, 0], [20])
    np.testing.assert_allclose(result.effort, [0], atol=1e-6)
    np.testing.assert_array_equal(result.diagnostics[8:10], [1, 1])


@pytest.mark.parametrize("bad_effort", [np.nan, 20.0, -101.0])
def test_success_status_does_not_bypass_raw_residual_validation(bad_effort):
    controller = make_filter()
    result = SimpleNamespace(
        is_success=lambda: True,
        GetSolution=lambda _: np.array([bad_effort]),
        get_solution_result=lambda: "kSolutionFound",
    )
    controller._solver = SimpleNamespace(Solve=lambda *args: result)
    with pytest.raises(CbfFailure, match="residual check") as error:
        controller.filter([0.5, 0], [20])
    assert "effort" in error.value.snapshot
    assert "residual" in error.value.snapshot


def ground_filter(*, limit=100, **kwargs):
    return SphereCbfFilter(
        model=slider(vertical=True, limit=limit),
        joints=("slide",),
        spheres=(Sphere("camera", "robot/body", (0, 0, 0), 0.1),),
        planes=(Plane("floor", (0, 0, 1), 0),),
        plane_pairs=(SpherePlanePair("camera", "floor", margin=0.02),),
        **kwargs,
    )


def test_ground_only_filter_projects_effort_with_signed_distance():
    controller = ground_filter()
    controller.validate_initial_state([0.5, 0])
    result = controller.filter([0.5, 0], [-20])
    # h=.38 m; hddot=u-9.81; u-9.81+25*.38 >= 0 means u >= .31.
    np.testing.assert_allclose(result.effort, [0.31], atol=1e-7)
    np.testing.assert_allclose(result.diagnostics[:3], [0.38, 0.38, 1.9])
    np.testing.assert_allclose(controller.clearances([0.5, 0]), [0.38])
    assert controller.pair_names == ("camera|floor",)
    assert controller.constraint_count == 1
    assert len(result.diagnostics) == 8


@pytest.mark.parametrize("state", [[0.11, 0], [0.5, -2]])
def test_ground_initial_domain_rejects_penetration_or_excessive_approach(state):
    with pytest.raises(CbfFailure, match="initial state") as error:
        ground_filter().validate_initial_state(state)
    assert "camera|floor" in error.value.snapshot["pairs"]


def test_ground_constraint_is_not_softened_when_effort_is_insufficient():
    controller = ground_filter(limit=2)
    controller.validate_initial_state([0.12, 0])
    with pytest.raises(CbfFailure, match="QP failed") as error:
        controller.filter([0.12, 0], [0])
    assert "camera|floor" in error.value.snapshot["pairs"]


def test_sphere_rows_precede_plane_rows_and_use_same_qp():
    controller = SphereCbfFilter(
        model=slider(vertical=True),
        joints=("slide",),
        spheres=(
            Sphere("camera", "robot/body", (0, 0, 0), 0.1),
            Sphere("ceiling", "world", (0, 0, 1), 0.1),
        ),
        pairs=(SpherePair("camera", "ceiling", margin=0),),
        planes=(Plane("floor", (0, 0, 1), 0.2),),
        plane_pairs=(SpherePlanePair("camera", "floor", margin=0.02),),
    )
    result = controller.filter([0.5, 0], [-20])
    assert controller.pair_names == ("camera|ceiling", "camera|floor")
    assert controller.constraint_count == 2
    np.testing.assert_allclose(result.effort, [5.31], atol=1e-7)
    np.testing.assert_allclose(result.diagnostics[:4], [0.3, 0.18, 0.21, 0.18])
    np.testing.assert_allclose(controller.clearances([0.5, 0]), [0.3, 0.18])
    assert np.all(result.diagnostics[6:8] >= -1e-6)


@pytest.mark.parametrize(
    ("planes", "plane_pairs", "message"),
    [
        (
            (Plane("floor", (0, 0, 1), 0),) * 2,
            (SpherePlanePair("camera", "floor"),),
            "Plane names",
        ),
        (
            (Plane("camera", (0, 0, 1), 0),),
            (SpherePlanePair("camera", "missing"),),
            "must not overlap",
        ),
        (
            (Plane("floor", (0, 0, 1), 0),),
            (SpherePlanePair("camera", "floor"),) * 2,
            "pairs must be unique",
        ),
        (
            (Plane("floor", (0, 0, 1), 0),),
            (SpherePlanePair("missing", "floor"),),
            "unknown sphere",
        ),
        (
            (Plane("floor", (0, 0, 1), 0),),
            (SpherePlanePair("camera", "missing"),),
            "unknown plane",
        ),
    ],
)
def test_plane_filter_rejects_ambiguous_or_unknown_references(
    planes, plane_pairs, message
):
    with pytest.raises(ValueError, match=message):
        SphereCbfFilter(
            model=slider(vertical=True),
            joints=("slide",),
            spheres=(Sphere("camera", "robot/body", (0, 0, 0), 0.1),),
            planes=planes,
            plane_pairs=plane_pairs,
        )


def test_ground_only_filter_prevents_penetration_in_discrete_simulation():
    # There is no physical contact geometry to stop the fall: the CBF alone
    # modifies the constant downward nominal command.
    builder = DiagramBuilder()
    plant = builder.AddSystem(slider(vertical=True, time_step=0.001))
    controller = ground_filter()
    safety = builder.AddSystem(SphereCbfSystem(filter=controller))
    nominal = builder.AddSystem(ConstantVectorSource([-20.0]))
    builder.Connect(
        plant.get_state_output_port(), safety.GetInputPort("estimated_state")
    )
    builder.Connect(nominal.get_output_port(), safety.GetInputPort("nominal_effort"))
    builder.Connect(safety.GetOutputPort("effort"), plant.get_actuation_input_port())
    logger = LogVectorOutput(safety.GetOutputPort("diagnostics"), builder)
    simulator = Simulator(builder.Build())
    context = plant.GetMyMutableContextFromRoot(simulator.get_mutable_context())
    plant.SetPositionsAndVelocities(context, [0.5, 0])
    controller.validate_initial_state([0.5, 0])
    simulator.AdvanceTo(1.0)
    diagnostics = logger.FindLog(simulator.get_context()).data()
    assert diagnostics[0].min() >= 0
    assert diagnostics[3].min() >= -controller.parameters.residual_tolerance
    assert diagnostics[5].max() > 1
    assert 0.12 < plant.GetPositions(context)[0] < 0.2


def test_clearance_system_reports_unsafe_state_without_solving():
    controller = ground_filter()
    controller._solver = SimpleNamespace(
        Solve=lambda *args: pytest.fail("Clearance observation must not solve a QP")
    )
    observer = CbfClearanceSystem(controller)
    context = observer.CreateDefaultContext()
    observer.GetInputPort("state").FixValue(context, [0.08, -0.5])
    np.testing.assert_allclose(
        observer.GetOutputPort("clearance").Eval(context), [-0.04]
    )
    observer.GetInputPort("state").FixValue(context, [0.5, 0])
    np.testing.assert_allclose(
        observer.GetOutputPort("clearance").Eval(context), [0.38]
    )


def test_shared_factory_preserves_custom_gains_and_plane_geometry():
    from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry
    from robo_arch.core.controllers.cbf.config import ProtectionParameters
    from robo_arch.core.controllers.cbf.drake import build_filter

    geometry = ProtectionGeometry(
        spheres=(Sphere("camera", "robot/body", (0, 0, 0), 0.1),),
        pairs=(),
        planes=(Plane("floor", (0, 0, 1), 0),),
        plane_pairs=(SpherePlanePair("camera", "floor", 0.02),),
    )
    parameters = ProtectionParameters(
        profiles={"camera": "package://robo_arch/synthetic/protection.yaml"},
        protected=("camera",),
        margin=0.02,
        alpha1=3,
        alpha2=4,
        residual_tolerance=1e-7,
    )
    # Resolved geometry is sufficient: building the runtime filter reads no assets.
    controller = build_filter(
        model=slider(vertical=True),
        joints=("slide",),
        geometry=geometry,
        parameters=parameters,
    )
    controller.validate_initial_state([0.5, 0])
    # h=.38 m; u >= 9.81 - (3*4)*.38 = 5.25 N.
    np.testing.assert_allclose(
        controller.filter([0.5, 0], [-20]).effort, [5.25], atol=1e-7
    )
    assert controller.parameters.residual_tolerance == 1e-7


@pytest.mark.parametrize("direction", [-1, 1])
def test_velocity_barrier_limits_outward_acceleration(direction):
    controller = make_filter(vertical=True, velocity_limit=0.5, damping=0.3)
    state = np.array([0.5, direction * 0.49])
    controller.validate_initial_state(state)
    result = controller.filter(state, [direction * 80.0])
    acceleration = result.effort[0] - 9.81 - 0.3 * state[1]
    np.testing.assert_allclose(acceleration, direction * 0.2, atol=1e-7)
    assert abs(state[1] + 0.001 * acceleration) < 0.5
    assert result.diagnostics.shape == (8,)
    with pytest.raises(CbfFailure, match="initial velocity"):
        controller.validate_initial_state(np.array([0.5, direction * 0.501]))


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
