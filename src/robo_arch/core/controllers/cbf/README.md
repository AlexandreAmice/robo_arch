# Sphere effort filter

`SphereCbfFilter` projects a nominal actuator effort onto hard second-order
sphere/sphere and sphere/plane separation constraints and actuator bounds.
The Drake adapter accepts a fixed-base fully actuated model, ordered `[q, v]`,
and any nominal effort source.
`wrap(nominal)` uses the existing `(state, time) -> effort` policy interface.
`SphereCbfSystem` has `estimated_state` and `nominal_effort` inputs, and `effort`
and `diagnostics` outputs sharing one native cache entry per context. Each
filter owns an independent dynamics context and optimization workspace; calls
on an individual filter must be sequential.

`config.ProtectionParameters` owns the reusable YAML fields: `profiles`,
`protected`, `exclude_frames`, `margin`, `alpha1`, `alpha2`, `backend`,
`compile_model`, `velocity_limit_gain`, and `residual_tolerance`. It selects no robot model, nominal controller or task.
`assembly.resolve_geometry(scene, parameters, ground=...)` resolves coverings,
fixed-object poses, exclusions and optional ground constraints into
`ProtectionGeometry`. Both configuration and geometry assembly work without
Drake or Isaac installed. Model-owned profile resources remain with their assets.

Given a loaded physical `scene`, a mapping of these protection settings,
an independent nominal `model`, ordered `joints` and a nominal effort callable:

```python
from robo_arch.core.controllers.cbf.assembly import resolve_geometry
from robo_arch.core.controllers.cbf.config import ProtectionParameters
from robo_arch.core.controllers.cbf.drake import build_filter

parameters = ProtectionParameters.model_validate(protection_settings)
geometry = resolve_geometry(scene, parameters, ground=True)
safety = build_filter(
    model=model, joints=joints, geometry=geometry, parameters=parameters
)
safety.validate_initial_state(initial_state)
protected_controller = safety.wrap(nominal_controller)
```

The caller explicitly selects ground protection to match its physical world.
`build_filter` uses Drake for CPU nominal dynamics and QP solving, independently
of which simulator supplies state. `CbfClearanceSystem` exposes the same geometry
for an unfiltered Drake baseline. `drake.py` also owns optional sphere and
ground overlays; using the callable filter does not require a viewer. The camera
scenario adds only its nominal-controller selection, motion task and evaluation.

Given `model` containing a mounted `camera/body` frame, ordered `joints`,
`initial_state`, and your `nominal_controller(state, time)` callable:

```python
from robo_arch.core.controllers.cbf.definition import (
    Plane,
    Sphere,
    SpherePair,
    SpherePlanePair,
)
from robo_arch.core.controllers.cbf.drake import SphereCbfFilter

safety = SphereCbfFilter(
    model=model,
    joints=joints,
    spheres=(
        Sphere("camera", "camera/body", (0.0, 0.0, 0.0), 0.06),
        Sphere("obstacle", "world", (0.8, 0.0, 0.4), 0.10),
    ),
    pairs=(SpherePair("camera", "obstacle", margin=0.02),),
    planes=(Plane("floor", (0.0, 0.0, 1.0), offset=0.0),),
    plane_pairs=(SpherePlanePair("camera", "floor", margin=0.02),),
)
safety.validate_initial_state(initial_state)
protected_controller = safety.wrap(nominal_controller)
effort = protected_controller(initial_state, 0.0)
```

Choose sphere centers/radii for your actual geometry; add named spheres and
pairs to protect further objects without changing the nominal controller.

Declare `Sphere(name, frame, center, radius)` in meters. Frames are `world` or
`model_instance/frame`, including nested instance names. World spheres are
fixed. Moving spheres must belong to the controller model. `SpherePair` names
two spheres and an additional nonnegative clearance margin; explicit pairs
also define exclusions. Declarations import no simulator SDK. Coverage profiles
and `load_sphere_profile` in `geometry.py` conservatively cover collision assets
using circumspheres of partitioned bounding boxes. These covers can be more
restrictive than the original geometry.
Fixed-object profiles must use a single-link collision asset in the object's
declared base frame. Assets with additional links, joints, frames or nested
models are rejected for fixed objects because their base-to-link transforms are
not resolved. Device profiles retain link-local frames and may have many links.
The caller is responsible for matching each profile to its physical instance.

`Plane(name, normal, offset)` declares a fixed world halfspace
`normal @ position >= offset`. Its normal must be finite and unit length;
the offset is in meters. `SpherePlanePair` selects the sphere, plane and extra
margin. Sphere and plane names share one namespace. Plane-only filters may omit
`pairs`; plane declarations alone do not add constraints without `plane_pairs`.
The physical world's ground and protection plane are separate declarations.

For center difference `d`, relative Jacobian `J`, bias acceleration `beta`,
and combined radius plus margin `R`, the filter uses
`h = d.T @ d - R**2`, `hdot = 2*d.T @ J @ v`, and
`hddot = 2*||J @ v||² + 2*d.T @ (J @ vdot + beta)`.
The QP enforces
`hddot + (alpha1 + alpha2)*hdot + alpha1*alpha2*h >= 0`
while minimizing `0.5*||effort - nominal_effort||²`.
Dynamics include mass, force-element contributions (including gravity and joint
damping), and Coriolis bias exactly once. Both moving centers contribute to
relative derivatives. This is a nominal contact-free model; external applied
or contact forces are not inferred.

For a fixed plane, use linear signed clearance
`h = normal @ center - offset - radius - margin`,
`hdot = normal @ J @ v` and `hddot = normal @ (J @ vdot + beta)`.
The same second-order inequality and effort limits apply; negative signed
distance remains a violation rather than becoming positive below the plane.

Finite model joint-velocity limits also enter the same QP. For each lower or
upper bound, the shared NumPy/Torch kernel enforces `hdot + velocity_limit_gain*h
>= 0`, where `h` is velocity slack and `hdot` comes from the same nominal joint
acceleration. The default gain is 20 s⁻¹. This prevents commanded acceleration
from relying on motion that a simulator's velocity clamp would remove. Infinite
bounds add no row. This continuous-time condition retains the sampled-control
and model-error limitations below. Joint-stop and other contact impulses remain
outside the nominal model.

Call `validate_initial_state` before simulation: velocity must lie within its
model bounds, and both `h >= 0` and
`psi1 = hdot + alpha1*h >= 0` must hold. `evaluate` exposes the same barrier rows
without solving a QP. `clearances` evaluates separation using positions alone
for an unfiltered baseline, avoiding unnecessary dynamics and derivatives.
Runtime assembly evaluates points together per frame and all pair constraints
as NumPy arrays; it retains every configured pair at every controller evaluation.
The Drake Clarabel QP solver uses scaled rows for conditioning; returned effort
is checked in original units. An exactly feasible nominal effort is returned
unchanged without invoking the solver, because it already minimizes the QP.
Solver failure, nonfinite output, or invalid effort/barrier residuals raise
`CbfFailure` with a named-pair failure snapshot. Constraints are never softened;
there is no fallback command or clipping after solving. Stop the run when the
exception propagates; stopping a simulator is not a hardware emergency stop.

For `P = constraint_count` enabled pairs, diagnostics contain five blocks of
length `P`, ordered as `pair_names`: sphere pairs first, then sphere-plane pairs.
Blocks contain clearance including margin, `h`, `psi1`, CBF residual, and
active-constraint indicators. Clearance is always in meters. Sphere-pair
`h`, `psi1`, and residual use m², m²/s and m²/s²; plane rows use m, m/s and
m/s² because their barrier is linear signed distance. The final three values are
effort correction norm (N m), measured solver duration (s), and success (1).
Solver duration is zero when an exactly feasible nominal effort passes through.
Failure emits an exception rather than a successful output vector. The initial
domain tolerance is `1e-10`; effort and residual tolerances default to `1e-6`
in their respective units. An active constraint has an enforced QP-row residual
at most ten times that tolerance. Diagnostic arrays and effort arrays are owned
outputs.

The continuous barrier condition evaluated by a discrete simulation is a
sampled approximation. It provides no independent guarantee between samples,
under infeasible torque bounds, inaccurate geometry/dynamics, or on hardware.
References: [high-order barriers](https://arxiv.org/abs/1903.04706) and
[Drake optimization results](https://drake.mit.edu/doxygen_cxx/classdrake_1_1solvers_1_1_mathematical_program_result.html).

## GPU execution

`isaac.build_filter(..., batch_size=N, device="cuda:0")` selects the batched
Torch/Moreau implementation with `backend: torch_moreau`. It consumes the same
`ProtectionGeometry`, gains and independent Drake model as the CPU factory.
`core.controllers.dynamics.drake` extracts constants once; the SDK-independent
`core.controllers.dynamics.torch.TensorModel` evaluates nominal dynamics on the
selected device. `barrier.py`, `layout.py` and `velocity.py` share the constraint
equations and indexing. `filter.py` shares initial-domain validation, full QP-row
assembly, command-rounding guards, acceptance checks and diagnostics across
NumPy and Torch. `drake.py` supplies Drake kinematics, Clarabel and native ports;
`isaac.py` constructs the tensor/Moreau filter for native effort commands.
`tensor.py` and `moreau.py` are numerical providers without simulator imports. There is no second
camera-specific CBF algorithm. `compile_model: true` optionally compiles those
same tensor dynamics with Torch; it incurs a substantial first-call compile and
keeps returned evaluations owned. Eager execution is the default.

`TensorCbfFilter.filter(state, nominal_effort)` accepts float64 CUDA arrays of
shape `[batch, 2*joints]` and `[batch, joints]`. `wrap(nominal)` protects any
compatible batched callable. An optional current model evaluation lets nominal
inverse dynamics and the barrier reuse the same calculations. The model supports
fixed-base scalar revolute/prismatic joints, welds and identity actuation;
unsupported force elements and floating bases fail explicitly.

`MoreauProjection` solves one fixed-structure batch with hard effort and barrier
constraints. Numerical matrices and solutions remain CUDA tensors. Moreau's
status metadata and explicit acceptance predicates still synchronize with the
host; this implementation is not synchronization-free. Any nonfinite input,
infeasible QP, failed model evaluation or rejected residual stops the entire
batch before commands are applied. No CPU solver or unsafe-command fallback is
used. Feasible nominal commands pass through exactly when no native precision
conversion is requested. The controller is not exposed as a differentiable layer.

Isaac uses `command_dtype=torch.float32`: barrier rows and actuator bounds are
tightened by a conservative effort-rounding bound, then the actual rounded
command is checked in original units. Model/barrier/QP calculations remain
float64. This addresses command conversion, not all PhysX/model error.
Diagnostics retain the scalar block layout with a leading batch axis; their
per-call timing slot is NaN because no per-step CUDA timing fence is introduced.
Reports omit that unmeasured value. Use the owning scenario benchmark for timing.
Active flags use the tightened QP rows before command rounding, so the added
rounding margin does not hide binding constraints. The residual block still
reports the original barriers evaluated on the actual rounded command.

Install the optional `cbf-gpu` and `test` groups in the independently locked
[Isaac profile](../../../../../third_party/isaac/README.md). Core configuration
and CPU execution do not import Torch or Moreau. GPU tests run in that profile;
Bazel's ordinary CPU environment does not validate CUDA execution.

```sh
uv run --project third_party/isaac --group cbf-gpu --group test python -m pytest -q \
  src/robo_arch/core/controllers/cbf/tests/test_moreau.py \
  src/robo_arch/core/controllers/cbf/tests/test_tensor_barrier.py \
  src/robo_arch/core/controllers/cbf/tests/test_tensor_filter.py \
  src/robo_arch/core/controllers/dynamics/tests
```

See the [camera scenario](../../../scenarios/camera_protection/README.md) for
batched physics, YAML selection and measured control throughput. Float64 CUDA
execution has not demonstrated a single-arm speedup over Drake. The same nominal,
contact-free and sampled-control limitations apply to both backends.

Public declaration and numerical contracts are maintained in source docstrings
and the [API catalogue](../../../../../docs/api/controllers.rst). The optional
Torch/Moreau runtime remains documented here and in the
[camera scenario](../../../scenarios/camera_protection/README.md); Isaac execution
uses the shared Isaac Lab scene and tensor buffers.
