"""Drake nominal dynamics, bounded torque QP, and native system adapter."""

from collections.abc import Callable
from time import perf_counter

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.geometry import Box, GeometryInstance, MakePhongIllustrationProperties
from pydrake.geometry import Sphere as DrakeSphere
from pydrake.math import RigidTransform
from pydrake.multibody.plant import MultibodyPlant
from pydrake.solvers import ClarabelSolver, MathematicalProgram, SolverOptions
from pydrake.systems.framework import BasicVector, Context, LeafSystem, ValueProducer

from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry
from robo_arch.core.controllers.cbf.barrier import (
    BarrierConstraint,
    BarrierConstraints,
    geometry_constraints,
)
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.cbf.definition import (
    CbfParameters,
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)
from robo_arch.core.controllers.cbf.errors import CbfFailure as CbfFailure
from robo_arch.core.controllers.cbf.filter import (
    FilterResult,
    project,
    validate_initial_state,
)
from robo_arch.core.controllers.cbf.layout import compile_geometry
from robo_arch.core.controllers.cbf.velocity import compile_velocity_bounds
from robo_arch.core.controllers.dynamics.drake import build_tensor_model
from robo_arch.core.worlds.drake.scene import DrakeScene


class SphereCbfFilter:
    """Filter nominal actuator efforts for one fixed-base, fully actuated robot.

    Owns its model context and optimization workspace. Calls on one instance
    are sequential; independent instances share only the immutable model.
    Unmodeled applied/contact forces are outside this nominal dynamics model.
    """

    def __init__(
        self,
        *,
        model: MultibodyPlant,
        spheres: tuple[Sphere, ...],
        pairs: tuple[SpherePair, ...] = (),
        joints: tuple[str, ...],
        parameters: CbfParameters | None = None,
        planes: tuple[Plane, ...] = (),
        plane_pairs: tuple[SpherePlanePair, ...] = (),
    ):
        if not isinstance(model, MultibodyPlant) or not model.is_finalized():
            raise ValueError("CBF requires a finalized Drake controller model")
        self.count = model.num_velocities()
        if (
            model.num_positions() != self.count
            or model.num_actuated_dofs() != self.count
            or self.count == 0
        ):
            raise ValueError("CBF requires a fixed-base fully actuated model")
        actuators = [
            model.get_joint_actuator(i) for i in model.GetJointActuatorIndices()
        ]
        if tuple(a.joint().name() for a in actuators) != joints:
            raise ValueError("CBF model actuator joint order does not match joints")
        if any(
            a.joint().num_positions() != 1
            or a.joint().num_velocities() != 1
            or a.joint().position_start() != i
            or a.joint().velocity_start() != i
            for i, a in enumerate(actuators)
        ):
            raise ValueError("CBF requires matching scalar q, v and actuator order")
        layout = compile_geometry(spheres, pairs, planes, plane_pairs)
        self._layout = layout
        self.model = model
        self.parameters = parameters if parameters is not None else CbfParameters()
        self.spheres = spheres
        self.pairs = pairs
        self.planes = planes
        self.plane_pairs = plane_pairs
        self.pair_names = layout.pair_names
        self.constraint_count = len(self.pair_names)
        self._velocity_bounds = compile_velocity_bounds(
            joints, model.GetVelocityLowerLimits(), model.GetVelocityUpperLimits()
        )
        self.velocity_bound_names = self._velocity_bounds.names
        self.qp_constraint_count = self.constraint_count + len(
            self._velocity_bounds.names
        )
        self.diagnostics_size = 5 * self.constraint_count + 3
        self._nominal = build_tensor_model(
            model,
            joints,
            tuple(s.frame for s in spheres),
            tuple(s.center for s in spheres),
            device="cpu",
        )
        self._state = None
        self._evaluation = None
        self._first = layout.first
        self._second = layout.second
        self._separation = layout.separation
        self._local_centers = layout.local_centers
        self._plane_spheres = layout.plane_spheres
        self._plane_normals = layout.plane_normals
        self._plane_offsets = layout.plane_offsets
        self._limits = np.asarray([a.effort_limit() for a in actuators])
        if not np.isfinite(self._limits).all() or np.any(self._limits <= 0):
            raise ValueError("CBF requires finite positive actuator effort limits")
        self._actuation = model.MakeActuationMatrix()
        if np.linalg.matrix_rank(self._actuation) != self.count:
            raise ValueError("CBF requires full-rank actuation")
        self._solver = ClarabelSolver()
        if not self._solver.available() or not self._solver.enabled():
            raise RuntimeError("CBF requires Drake's Clarabel solver")
        self._program = MathematicalProgram()
        self._effort = self._program.NewContinuousVariables(self.count, "effort")
        self._cost = self._program.AddQuadraticCost(
            np.eye(self.count), np.zeros(self.count), self._effort
        ).evaluator()
        self._program.AddBoundingBoxConstraint(
            -self._limits, self._limits, self._effort
        )
        self._inequality = self._program.AddLinearConstraint(
            np.zeros((self.qp_constraint_count, self.count)),
            np.zeros(self.qp_constraint_count),
            np.full(self.qp_constraint_count, np.inf),
            self._effort,
        ).evaluator()
        self._options = SolverOptions()
        for name in ("tol_feas", "tol_gap_abs", "tol_gap_rel"):
            self._options.SetOption(self._solver.id(), name, 1e-10)
        self._options.SetOption(self._solver.id(), "max_iter", 200)

    def _set_state(self, state: np.ndarray, time: float) -> np.ndarray:
        state = np.asarray(state, dtype=float)
        if state.shape != (2 * self.count,) or not np.isfinite(state).all():
            raise ValueError("CBF state must contain finite ordered [q, v]")
        if not np.isfinite(time):
            raise ValueError("CBF time must be finite")
        if self._state is None or not np.array_equal(state, self._state):
            self._evaluation = self._nominal.evaluate_numpy(state[None, :])
            self._state = state.copy()
        return state

    def _kinematics(self, *, derivatives: bool):
        data = self._evaluation
        return (
            data.positions[0],
            data.jacobians[0] if derivatives else None,
            data.bias_accelerations[0] if derivatives else None,
        )

    def clearances(self, state: np.ndarray, time: float = 0.0) -> np.ndarray:
        """Inspect pair clearances without computing derivatives or dynamics."""
        self._set_state(state, time)
        positions, _, _ = self._kinematics(derivatives=False)
        displacement = positions[self._first] - positions[self._second]
        clearances = np.linalg.norm(displacement, axis=1) - self._separation
        if not self.plane_pairs:
            return clearances
        plane_clearances = (
            np.einsum("pi,pi->p", self._plane_normals, positions[self._plane_spheres])
            - self._plane_offsets
        )
        return np.concatenate((clearances, plane_clearances))

    def _constraints(self, state: np.ndarray, time: float) -> BarrierConstraints:
        state = self._set_state(state, time)
        data = self._evaluation
        acceleration = np.column_stack(
            (
                data.acceleration_drift[0],
                data.acceleration_control[0],
            )
        )
        positions, jacobians, biases = self._kinematics(derivatives=True)
        self._velocity_coefficient, self._velocity_constant = (
            self._velocity_bounds.constraints(
                state[self.count :],
                acceleration[:, 0],
                acceleration[:, 1:],
                self.parameters.velocity_limit_gain,
            )
        )
        return geometry_constraints(
            positions=positions,
            jacobians=jacobians,
            bias_accelerations=biases,
            velocity=state[self.count :],
            acceleration_drift=acceleration[:, 0],
            acceleration_control=acceleration[:, 1:],
            layout=self._layout,
            parameters=self.parameters,
        )

    def evaluate(
        self, state: np.ndarray, time: float = 0.0
    ) -> tuple[BarrierConstraint, ...]:
        """Inspect barrier geometry/dynamics without solving or applying effort."""
        return self._constraints(state, time).rows()

    def validate_initial_state(self, state: np.ndarray, time: float = 0.0) -> None:
        """Reject separation or first-order barrier violations before starting."""
        rows = self._constraints(state, time)
        validate_initial_state(
            rows,
            self._velocity_bounds.slack(np.asarray(state)[self.count :]),
            pair_names=self.pair_names,
            velocity_names=self.velocity_bound_names,
            snapshot=lambda: {"time": time, "state": np.asarray(state).tolist()},
        )

    def filter(
        self, state: np.ndarray, nominal_effort: np.ndarray, time: float = 0.0
    ) -> FilterResult:
        """Return bounded effort or raise; never soften a barrier or fall back."""
        nominal = np.asarray(nominal_effort, dtype=float)
        if nominal.shape != (self.count,) or not np.isfinite(nominal).all():
            raise ValueError("CBF nominal effort must be finite in actuator order")
        constraints = self._constraints(state, time)
        result = None
        solve_duration = 0.0

        def snapshot() -> dict:
            return {
                "time": time,
                "state": np.asarray(state).tolist(),
                "nominal_effort": nominal.tolist(),
                "velocity_bounds": self._velocity_bounds.names,
                "pairs": {
                    name: {"clearance": c.clearance, "h": c.h, "psi1": c.psi1}
                    for name, c in zip(self.pair_names, constraints.rows(), strict=True)
                },
                "solver_status": (
                    "nominal_feasible"
                    if result is None
                    else str(result.get_solution_result())
                ),
                "solve_duration": solve_duration,
            }

        def solve(coefficient, constant, nominal):
            nonlocal result, solve_duration
            if not np.isfinite(coefficient).all() or not np.isfinite(constant).all():
                raise CbfFailure("nonfinite dynamics", **snapshot())
            # Preserve feasible nominal efforts exactly.
            if np.all(np.abs(nominal) <= self._limits) and np.all(
                coefficient @ nominal + constant >= 0
            ):
                return nominal.copy()
            self._cost.UpdateCoefficients(np.eye(self.count), -nominal)
            scale = np.maximum(
                1.0, np.maximum(np.max(np.abs(coefficient), axis=1), np.abs(constant))
            )
            self._inequality.UpdateCoefficients(
                coefficient / scale[:, None],
                -constant / scale,
                np.full(self.qp_constraint_count, np.inf),
            )
            start = perf_counter()
            result = self._solver.Solve(self._program, None, self._options)
            solve_duration = perf_counter() - start
            if not result.is_success():
                raise CbfFailure("QP failed", **snapshot())
            return result.GetSolution(self._effort)

        return project(
            rows=constraints,
            velocity_coefficient=self._velocity_coefficient,
            velocity_constant=self._velocity_constant,
            velocity_slack=self._velocity_bounds.slack(np.asarray(state)[self.count :]),
            nominal=nominal,
            limits=self._limits,
            solve=solve,
            tolerance=self.parameters.residual_tolerance,
            solve_duration=lambda: solve_duration,
            snapshot=snapshot,
        )

    def wrap(
        self, nominal: Callable[[np.ndarray, float], np.ndarray]
    ) -> Callable[[np.ndarray, float], np.ndarray]:
        """Adapt any nominal effort callable with the existing policy signature."""

        def command(state: np.ndarray, time: float) -> np.ndarray:
            return self.filter(state, nominal(state, time), time).effort

        return command


class SphereCbfSystem(LeafSystem):
    """Direct-feedthrough effort and diagnostics, sharing a native cache entry."""

    def __init__(self, *, filter: SphereCbfFilter):
        super().__init__()
        self.filter = filter
        self._state = self.DeclareVectorInputPort("estimated_state", 2 * filter.count)
        self._nominal = self.DeclareVectorInputPort("nominal_effort", filter.count)
        self._result = self.DeclareCacheEntry(
            "CBF effort and diagnostics",
            ValueProducer(
                lambda: AbstractValue.Make(
                    FilterResult(
                        np.zeros(filter.count), np.zeros(filter.diagnostics_size)
                    )
                ),
                self._calculate,
            ),
        )
        self.DeclareVectorOutputPort(
            "effort",
            BasicVector(filter.count),
            self._effort_output,
            prerequisites_of_calc={self._result.ticket()},
        )
        self.DeclareVectorOutputPort(
            "diagnostics",
            BasicVector(filter.diagnostics_size),
            self._diagnostics_output,
            prerequisites_of_calc={self._result.ticket()},
        )

    def _calculate(self, context, value) -> None:
        value.set_value(
            self.filter.filter(
                self._state.Eval(context),
                self._nominal.Eval(context),
                context.get_time(),
            )
        )

    def _effort_output(self, context, output) -> None:
        output.SetFromVector(self._result.EvalAbstract(context).get_value().effort)

    def _diagnostics_output(self, context, output) -> None:
        output.SetFromVector(self._result.EvalAbstract(context).get_value().diagnostics)


def build_filter(
    *,
    model: MultibodyPlant,
    joints: tuple[str, ...],
    geometry: ProtectionGeometry,
    parameters: ProtectionParameters,
) -> SphereCbfFilter:
    """Build the CPU effort filter from shared geometry and validated YAML settings.

    The model supplies nominal dynamics; the calling world supplies actual state.
    Call ``validate_initial_state`` before applying commands in that world.
    """
    return SphereCbfFilter(
        model=model,
        joints=joints,
        spheres=geometry.spheres,
        pairs=geometry.pairs,
        planes=geometry.planes,
        plane_pairs=geometry.plane_pairs,
        parameters=CbfParameters(
            alpha1=parameters.alpha1,
            alpha2=parameters.alpha2,
            velocity_limit_gain=parameters.velocity_limit_gain,
            residual_tolerance=parameters.residual_tolerance,
        ),
    )


class CbfClearanceSystem(LeafSystem):
    """Expose signed clearance in metres, in pair_names order, without a QP.

    The state input contains the filter's ordered [q, v]. Unsafe states produce
    negative clearances so the same observer also serves unfiltered baselines.
    """

    def __init__(self, cbf: SphereCbfFilter):
        super().__init__()
        self.cbf = cbf
        self.state = self.DeclareVectorInputPort("state", 2 * cbf.count)
        self.DeclareVectorOutputPort(
            "clearance", BasicVector(len(cbf.pair_names)), self.output
        )

    def output(self, context: Context, output: BasicVector) -> None:
        output.SetFromVector(
            self.cbf.clearances(self.state.Eval(context), context.get_time())
        )


def add_sphere_illustrations(
    scene: DrakeScene, spheres: tuple[Sphere, ...], protected: tuple[str, ...]
) -> None:
    """Attach transparent illustration-only geometry; no contact or RGB-D changes."""
    source = scene.plant.get_source_id()
    for sphere in spheres:
        color = (
            [0.1, 0.6, 1.0, 0.24]
            if any(sphere.name.startswith(name + "/") for name in protected)
            else [0.9, 0.55, 0.15, 0.12]
        )
        geometry = GeometryInstance(
            RigidTransform(sphere.center),
            DrakeSphere(sphere.radius),
            "protection/" + sphere.name,
        )
        properties = MakePhongIllustrationProperties(color)
        properties.AddProperty("meshcat", "accepting", "protections")
        geometry.set_illustration_properties(properties)
        if sphere.frame == "world":
            scene.scene_graph.RegisterAnchoredGeometry(source, geometry)
        else:
            instance, frame_name = sphere.frame.rsplit("/", 1)
            model = scene.plant.GetModelInstanceByName(instance)
            frame = scene.plant.GetFrameByName(frame_name, model)
            geometry.set_pose(
                frame.GetFixedPoseInBodyFrame() @ RigidTransform(sphere.center)
            )
            scene.scene_graph.RegisterGeometry(
                source,
                scene.plant.GetBodyFrameIdOrThrow(frame.body().index()),
                geometry,
            )


def add_ground_illustrations(scene: DrakeScene, margin: float) -> None:
    """Preview the infinite sphere-bottom limit with a finite 10 m cyan grid.

    Grid and border upper faces are exactly at z=margin. The translucent fill
    sits 0.2 mm below them to avoid coplanar surfaces. All geometry is anchored
    illustration in the protections layer; the physical floor remains at z=0.
    """
    source = scene.plant.get_source_id()

    def add(
        name: str,
        size: tuple[float, float, float],
        center: tuple[float, float, float],
        color: tuple[float, float, float, float],
    ) -> None:
        geometry = GeometryInstance(
            RigidTransform(center), Box(*size), "protection/ground/" + name
        )
        properties = MakePhongIllustrationProperties(color)
        properties.AddProperty("meshcat", "accepting", "protections")
        geometry.set_illustration_properties(properties)
        scene.scene_graph.RegisterAnchoredGeometry(source, geometry)

    add(
        "fill",
        (10.0, 10.0, 0.001),
        (0.0, 0.0, margin - 0.0007),
        (0.05, 0.75, 0.95, 0.10),
    )
    for index in range(1, 20):
        offset = -5.0 + 0.5 * index
        add(
            f"grid/x_{index}",
            (10.0, 0.003, 0.0005),
            (0.0, offset, margin - 0.00025),
            (0.05, 0.75, 0.95, 0.65),
        )
        add(
            f"grid/y_{index}",
            (0.003, 10.0, 0.0005),
            (offset, 0.0, margin - 0.00025),
            (0.05, 0.75, 0.95, 0.65),
        )
    for index, offset in enumerate((-4.996, 4.996)):
        add(
            f"border/x_{index}",
            (10.0, 0.008, 0.001),
            (0.0, offset, margin - 0.0005),
            (0.05, 0.8, 1.0, 0.85),
        )
        add(
            f"border/y_{index}",
            (0.008, 10.0, 0.001),
            (offset, 0.0, margin - 0.0005),
            (0.05, 0.8, 1.0, 0.85),
        )
