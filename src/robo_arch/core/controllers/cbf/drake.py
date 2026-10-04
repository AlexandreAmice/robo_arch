"""Drake nominal dynamics, bounded torque QP, and native system adapter."""

from collections.abc import Callable
from dataclasses import dataclass
from time import perf_counter

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable, MultibodyForces
from pydrake.solvers import ClarabelSolver, MathematicalProgram, SolverOptions
from pydrake.systems.framework import BasicVector, Context, LeafSystem, ValueProducer

from robo_arch.core.controllers.cbf.assembly import ProtectionGeometry
from robo_arch.core.controllers.cbf.barrier import (
    BarrierConstraint,
    BarrierConstraints,
    plane_constraints,
    sphere_constraints,
)
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.cbf.definition import (
    CbfParameters,
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)


@dataclass(frozen=True)
class FilterResult:
    """Owned output arrays; diagnostics use the block layout in the README."""

    effort: np.ndarray
    diagnostics: np.ndarray


class CbfFailure(RuntimeError):
    """Stop the calling run, preserving a numerical failure snapshot."""

    def __init__(self, reason: str, **snapshot):
        self.snapshot = snapshot
        super().__init__(f"CBF {reason}: {snapshot}")


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
        if not spheres or not (pairs or plane_pairs):
            raise ValueError("CBF requires spheres and enabled pairs")
        names = [s.name for s in spheres]
        if len(set(names)) != len(names):
            raise ValueError("Sphere names must be unique")
        plane_names = [plane.name for plane in planes]
        if len(set(plane_names)) != len(plane_names):
            raise ValueError("Plane names must be unique")
        if set(names).intersection(plane_names):
            raise ValueError("Sphere and plane names must not overlap")
        keys = [frozenset((p.first, p.second)) for p in pairs]
        if len(set(keys)) != len(keys):
            raise ValueError("Sphere pairs must be unique, including reverse pairs")
        if any(not key.issubset(names) for key in keys):
            raise ValueError("Sphere pair references an unknown sphere")
        plane_keys = [(pair.sphere, pair.plane) for pair in plane_pairs]
        if len(set(plane_keys)) != len(plane_keys):
            raise ValueError("Sphere-plane pairs must be unique")
        if any(pair.sphere not in names for pair in plane_pairs):
            raise ValueError("Sphere-plane pair references an unknown sphere")
        if any(pair.plane not in plane_names for pair in plane_pairs):
            raise ValueError("Sphere-plane pair references an unknown plane")
        self.model = model
        self.parameters = parameters if parameters is not None else CbfParameters()
        self.spheres = spheres
        self.pairs = pairs
        self.planes = planes
        self.plane_pairs = plane_pairs
        self.pair_names = tuple(f"{p.first}|{p.second}" for p in pairs) + tuple(
            f"{p.sphere}|{p.plane}" for p in plane_pairs
        )
        if len(set(self.pair_names)) != len(self.pair_names):
            raise ValueError("Protection pair display names must be unique")
        self.constraint_count = len(self.pair_names)
        self.diagnostics_size = 5 * self.constraint_count + 3
        self._context = model.CreateDefaultContext()
        self._frames = {}
        for sphere in spheres:
            if sphere.frame == "world":
                self._frames[sphere.name] = model.world_frame()
            else:
                if "/" not in sphere.frame:
                    raise ValueError("Sphere frame must be world or instance/frame")
                instance_name, frame_name = sphere.frame.rsplit("/", 1)
                instance = model.GetModelInstanceByName(instance_name)
                self._frames[sphere.name] = model.GetFrameByName(frame_name, instance)
        sphere_indices = {sphere.name: i for i, sphere in enumerate(spheres)}
        self._first = np.array([sphere_indices[p.first] for p in pairs], dtype=int)
        self._second = np.array([sphere_indices[p.second] for p in pairs], dtype=int)
        self._separation = np.array(
            [
                spheres[i].radius + spheres[j].radius + pair.margin
                for i, j, pair in zip(self._first, self._second, pairs, strict=True)
            ]
        )
        self._local_centers = np.array([s.center for s in spheres], dtype=float)
        plane_by_name = {plane.name: plane for plane in planes}
        self._plane_spheres = np.array(
            [sphere_indices[pair.sphere] for pair in plane_pairs], dtype=int
        )
        self._plane_normals = np.array(
            [plane_by_name[pair.plane].normal for pair in plane_pairs], dtype=float
        ).reshape(-1, 3)
        self._plane_offsets = np.array(
            [
                plane_by_name[pair.plane].offset + spheres[index].radius + pair.margin
                for pair, index in zip(plane_pairs, self._plane_spheres, strict=True)
            ]
        )
        grouped = {}
        for i, sphere in enumerate(spheres):
            if sphere.frame != "world":
                grouped.setdefault(sphere.frame, []).append(i)
        self._frame_groups = tuple(
            (
                self._frames[spheres[indices[0]].name],
                np.array(indices),
                np.asfortranarray(self._local_centers[indices].T),
            )
            for indices in grouped.values()
        )
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
            np.zeros((self.constraint_count, self.count)),
            np.zeros(self.constraint_count),
            np.full(self.constraint_count, np.inf),
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
        self._context.SetTime(time)
        self.model.SetPositionsAndVelocities(self._context, state)
        return state

    def _kinematics(
        self, *, derivatives: bool
    ) -> tuple[np.ndarray, np.ndarray | None, np.ndarray | None]:
        model, context = self.model, self._context
        world = model.world_frame()
        positions = self._local_centers.copy()
        jacobians = (
            np.zeros((len(self.spheres), 3, self.count)) if derivatives else None
        )
        biases = np.zeros((len(self.spheres), 3)) if derivatives else None
        for frame, indices, points in self._frame_groups:
            positions[indices] = model.CalcPointsPositions(
                context, frame, points, world
            ).T
            if derivatives:
                jacobians[indices] = model.CalcJacobianTranslationalVelocity(
                    context, JacobianWrtVariable.kV, frame, points, world, world
                ).reshape(len(indices), 3, self.count)
                biases[indices] = model.CalcBiasTranslationalAcceleration(
                    context, JacobianWrtVariable.kV, frame, points, world, world
                ).T
        return positions, jacobians, biases

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
        model, context = self.model, self._context
        forces = MultibodyForces(model)
        # This API includes gravity AND joint damping; do not add either again.
        model.CalcForceElementsContribution(context, forces)
        drift_force = -model.CalcInverseDynamics(context, np.zeros(self.count), forces)
        mass = model.CalcMassMatrix(context)
        acceleration = np.linalg.solve(
            mass, np.column_stack((drift_force, self._actuation))
        )
        positions, jacobians, biases = self._kinematics(derivatives=True)
        first, second = self._first, self._second
        sphere_rows = sphere_constraints(
            displacement=positions[first] - positions[second],
            relative_jacobian=jacobians[first] - jacobians[second],
            relative_bias_acceleration=biases[first] - biases[second],
            velocity=state[self.count :],
            acceleration_drift=acceleration[:, 0],
            acceleration_control=acceleration[:, 1:],
            separation=self._separation,
            parameters=self.parameters,
        )
        if not self.plane_pairs:
            return sphere_rows
        plane_rows = plane_constraints(
            positions=positions[self._plane_spheres],
            jacobians=jacobians[self._plane_spheres],
            bias_accelerations=biases[self._plane_spheres],
            normals=self._plane_normals,
            offsets_with_radius=self._plane_offsets,
            velocity=state[self.count :],
            acceleration_drift=acceleration[:, 0],
            acceleration_control=acceleration[:, 1:],
            parameters=self.parameters,
        )
        if not self.pairs:
            return plane_rows
        return BarrierConstraints(
            coefficient=np.vstack((sphere_rows.coefficient, plane_rows.coefficient)),
            constant=np.concatenate((sphere_rows.constant, plane_rows.constant)),
            clearance=np.concatenate((sphere_rows.clearance, plane_rows.clearance)),
            h=np.concatenate((sphere_rows.h, plane_rows.h)),
            psi1=np.concatenate((sphere_rows.psi1, plane_rows.psi1)),
        )

    def evaluate(
        self, state: np.ndarray, time: float = 0.0
    ) -> tuple[BarrierConstraint, ...]:
        """Inspect barrier geometry/dynamics without solving or applying effort."""
        return self._constraints(state, time).rows()

    def validate_initial_state(self, state: np.ndarray, time: float = 0.0) -> None:
        """Reject separation or first-order barrier violations before starting."""
        constraints = self.evaluate(state, time)
        invalid = {
            name: {"h": c.h, "psi1": c.psi1}
            for name, c in zip(self.pair_names, constraints, strict=True)
            if c.h < -1e-10 or c.psi1 < -1e-10
        }
        if invalid:
            raise CbfFailure(
                "inadmissible initial state",
                time=time,
                state=np.asarray(state).tolist(),
                pairs=invalid,
            )

    def filter(
        self, state: np.ndarray, nominal_effort: np.ndarray, time: float = 0.0
    ) -> FilterResult:
        """Return bounded effort or raise; never soften a barrier or fall back."""
        nominal = np.asarray(nominal_effort, dtype=float)
        if nominal.shape != (self.count,) or not np.isfinite(nominal).all():
            raise ValueError("CBF nominal effort must be finite in actuator order")
        constraints = self._constraints(state, time)
        coefficient = constraints.coefficient
        constant = constraints.constant
        if not np.isfinite(coefficient).all() or not np.isfinite(constant).all():
            raise CbfFailure(
                "nonfinite dynamics", time=time, state=np.asarray(state).tolist()
            )
        # A feasible nominal effort is the exact unconstrained minimizer. This
        # also preserves arbitrary nominal controllers without numerical drift.
        nominal_feasible = np.all(np.abs(nominal) <= self._limits) and np.all(
            coefficient @ nominal + constant >= 0
        )
        result = None
        solve_duration = 0.0
        if nominal_feasible:
            effort = nominal.copy()
        else:
            self._cost.UpdateCoefficients(np.eye(self.count), -nominal)
            scale = np.maximum(
                1.0, np.maximum(np.max(np.abs(coefficient), axis=1), np.abs(constant))
            )
            self._inequality.UpdateCoefficients(
                coefficient / scale[:, None],
                -constant / scale,
                np.full(self.constraint_count, np.inf),
            )
            start = perf_counter()
            result = self._solver.Solve(self._program, None, self._options)
            solve_duration = perf_counter() - start

        def snapshot() -> dict:
            return {
                "time": time,
                "state": np.asarray(state).tolist(),
                "nominal_effort": nominal.tolist(),
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

        if result is not None and not result.is_success():
            raise CbfFailure("QP failed", **snapshot())
        if result is not None:
            effort = result.GetSolution(self._effort)
        residual = coefficient @ effort + constant
        tolerance = self.parameters.residual_tolerance
        if (
            not np.isfinite(effort).all()
            or not np.isfinite(residual).all()
            or np.any(residual < -tolerance)
            or np.any(np.abs(effort) > self._limits + tolerance)
        ):
            raise CbfFailure(
                "QP residual check failed",
                **snapshot(),
                effort=effort.tolist(),
                residual=residual.tolist(),
            )
        diagnostics = np.concatenate(
            (
                constraints.clearance,
                constraints.h,
                constraints.psi1,
                residual,
                (residual <= 10 * tolerance).astype(float),
                [np.linalg.norm(effort - nominal), solve_duration, 1.0],
            )
        )
        return FilterResult(effort.copy(), diagnostics)

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
