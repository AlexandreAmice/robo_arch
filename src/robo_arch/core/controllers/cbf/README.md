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
`protected`, `exclude_frames`, `margin`, `alpha1`, `alpha2`, and
`residual_tolerance`. It selects no robot model, nominal controller or task.
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
for an unfiltered Drake baseline. `visualization` owns optional Drake sphere and
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

Call `validate_initial_state` before simulation: both `h >= 0` and
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
in their respective units. An active constraint has residual at most ten
times that tolerance. Diagnostic arrays and effort arrays are owned outputs.

The continuous barrier condition evaluated by a discrete simulation is a
sampled approximation. It provides no independent guarantee between samples,
under infeasible torque bounds, inaccurate geometry/dynamics, or on hardware.
References: [high-order barriers](https://arxiv.org/abs/1903.04706) and
[Drake optimization results](https://drake.mit.edu/doxygen_cxx/classdrake_1_1solvers_1_1_mathematical_program_result.html).

## GPU execution

The current implementation is scalar CPU control. GPU physics alone does not
move NumPy arrays, the Drake model, or the Clarabel QP onto the GPU. A GPU path
needs the following work; the backend and solver have not been selected:

- Batched state and effort buffers that stay on the device, including nominal
  control, reset masks and independent solver state per environment. Replace the
  current Isaac NumPy callback loop and per-step host logging. Omni Physics
  provides [Torch/Warp tensor frontends](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/107.0/extensions/runtime/source/omni.physics.tensors/docs/api/python.html);
  supported operations need checking against the pinned vendor environment.
- GPU nominal kinematics and dynamics from the shared model assets: mounted
  inertias, sphere-point Jacobians and bias acceleration, mass, gravity,
  Coriolis and damping. Reading simulator dynamics directly would be a named
  simulation approximation rather than the independent deployment model.
- Batched sphere/plane equations and a GPU solver for the small, hard-constrained
  effort QPs. Preserve effort bounds, infeasibility detection and checks in
  original units; copying every QP back to CPU would retain a synchronization
  bottleneck. A bounded-QP prototype is an early feasibility benchmark.
- Compare GPU kinematics, dynamics, constraints and commands against the current
  float64 CPU implementation, including moving pairs, ground, infeasibility and
  resets. Select precision, scaling and tolerances from those measurements.
- Measure both control latency and environment-step throughput across batch
  sizes, separating warmup/rendering from computation and accounting for copies
  and synchronization. A single six-joint arm has no demonstrated GPU speedup.

The protection declarations and geometry setup are shared inputs to that work;
this separation does not itself provide batched or GPU execution.
