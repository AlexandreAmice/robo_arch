# Camera protection

This example mounts three D435 housings on a UR7e's forearm and two wrist
links, with three fixed box obstacles. A nominal joint controller attempts an
obstructed target, then retreats. A torque CBF filter changes the command to keep
the configured camera sphere coverings clear of obstacles, the ground, other
cameras and nonexcluded robot links. Success requires intervention during the
unsafe phase, nonnegative clearance within numerical tolerance, accepted QP
residuals and a
successful retreat. Reaching the obstructed target is not required.
Approach and retreat each use a 0.8 s quintic position blend with consistent
desired velocity; the unsafe target is held until 3 s. Clearance evaluation
allows 10 micrometres of numerical tolerance around zero.

```sh
uv run src/robo_arch/scenarios/camera_protection/run.py --no-browser
uv run src/robo_arch/scenarios/camera_protection/run.py --baseline --no-browser
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --inspect recordings/camera_protection_filtered.json --visualization live_and_record
```

The default writes native Meshcat HTML, resolved-input JSON, measured NPZ and a
clearance/correction PNG under `recordings/`. `--visualization off` disables the
viewer. Inspection creates separate artifacts and rejects changed installed
source/assets or dependency versions; the original HTML and traces remain usable.
Failures retain their snapshot and available partial traces/playback. A failed
or infeasible QP raises immediately and stops simulation; no nominal fallback or
softened safety constraint is used.

Blue transparent spheres enclose cameras; orange spheres show counterpart
coverings. All coverings, including the fixed boxes, are in Meshcat's
`protections` layer, hidden by default; enable its visibility in the viewer tree.
Live viewing also
provides a `protections α` slider; native standalone HTML omits server-backed
sliders. The solid robots, cameras and boxes remain in the ordinary illustration
layer. These illustration-only overlays do not create physical contacts or appear
in RGB-D observations. Observations are disabled by default to keep this dynamics
example inexpensive; physical camera bodies, collisions
and inertia remain present. Set `sensors_enabled: true` for ideal Drake RGB-D.

The assembly YAML owns nominal mounts. Each device/object owns `protection.yaml`,
which selects its packaged collision asset and maximum AABB cell size. Every
complete cell is enclosed by its circumscribed sphere, covering collision-mesh
interiors as well as surfaces. This conservative approximation includes space
outside the actual object; finer subdivision trades more constraints for tighter
coverage. The sphere profile is independent of the scene instance and controller.
The profile loader supports OBJ meshes and boxes in URDF/SDF assets; unsupported
shapes and SDF frame semantics raise errors. The core filter also accepts explicit
frame-attached spheres without loading a profile.

Scenario `autonomy.parameters` selects profiles by physical instance, protected
instances, margin in metres, CBF gains and explicit frame-pair exclusions.
These protection fields use the shared `core.controllers.cbf.config` schema;
shared assembly resolves their geometry independently of a simulator SDK.
The scenario adds its arm/nominal-controller selection and motion task. Existing
YAML keys are unchanged; another scenario can use the same protection schema,
geometry resolver and filter without importing camera-protection code.
Within this scenario, `setup.py` resolves the arm, task, coverings and recorded
geometry and validates target limits for both worlds. `reference.py` supplies the
same motion reference; `drake.py` wires native ports and `isaac.py` supplies the
batched callback. Simulator adapters do not carry separate task definitions.
Only the three configured camera/mount-link pairs are excluded. Other links of
the same arm remain protected against camera contact. Robot–obstacle and general
robot self-collision are outside this example's selected pair set. Cables and
mounting brackets are not modeled, and mounts are not measured calibrations.
When `world.ground` is enabled, every protected sphere also gets a signed-distance
barrier against the infinite z=0 ground plane, including the configured margin.
These constraints use the same bounded-effort QP and failure handling as sphere
pairs. They protect the selected instances, not the unselected arm links.
Disabling the world floor also removes these plane constraints.

The `protections` layer includes a cyan ground grid whose upper surface is at
the configured margin (10 mm above the gray physical floor by default). It marks
the minimum allowed height of each protected sphere's **bottom**, not its center.
A translucent 10×10 m surface previews this boundary; the actual barrier is
infinite. The default box-avoidance motion stays far above the floor. Use the
ground-approach test below to see a sphere approach the cyan boundary, stop and
retreat. Ground-clearance plots report distance beyond the safety margin: zero
is the boundary, positive is clear, and negative is a violation.
Orange shading identifies active ground constraints; the separate torque plot
includes corrections from all constraints. Plot titles report the minimum
modeled sphere-to-floor gap before subtracting the margin.

The default nominal controller is `joint_tracking`; its `nominal` gains are
acceleration-feedback gains. `nominal_controller: joint_pd` instead selects the
existing native PD controller, with torque-feedback gains in that mapping.
The tested PD gains are `kp: [40, 80, 50, 8, 3, 1]` and
`kd: [13, 20, 13, 1, 0.5, 0.08]` in N m/rad and N m s/rad.
Direct scenario execution refreshes native PD automatically, as described
in the [native development workflow](../../../../docs/build_and_layout.md#build-and-python-workflow).
Physical assembly is unchanged. The reusable [CBF package](../../core/controllers/cbf/README.md)
also accepts any compatible nominal effort output through native ports or its
callable wrapper; it does not need camera-specific code.

Trace `cbf/diagnostics` follows the controller's documented block layout and
`geometry.pair_names` in JSON: sphere pairs first, then sphere–plane pairs.
The JSON includes plane geometry and associations; results separately report
`sphere_pair_count`, `plane_pair_count` and `minimum_ground_clearance_m`.
`pair_count` counts all constraint rows. Clearance includes the configured safety
margin.
`nominal_effort` and `commanded_effort` are in declared actuator order and N m;
`arm/q` and `arm/v` are radians and rad/s. Each custom channel has its own
`<channel>/times` array in seconds. Solver duration is measured wall time, not a
real-time guarantee. The baseline succeeds when it demonstrates a clearance
violation using the same geometry and reference.

Results report `simulation_wall_seconds` and `realtime_rate` (simulated seconds
per wall-clock second), excluding startup and artifact export. The default
checks 125 sphere pairs, three ground constraints and 12 joint-velocity bounds at 1 kHz. Pair algebra is
vectorized and point queries are grouped by frame; the remaining work includes
dynamics, an effort QP when
the nominal command is unsafe, logging and physics. Wall-clock rate also depends
on other processes; increasing `target_realtime_rate` cannot accelerate this work.

The CBF uses privileged simulated state, fixed known obstacle poses and nominal
rigid-body dynamics. The 1 ms simulation samples a continuous-time barrier law;
its recorded margin is measured evidence, not a guarantee between samples or
under unmodeled contact forces. Hardware and moving obstacles are not implemented. The optional Isaac GPU path
below reuses the same profiles, task reference, barrier equations and evaluation.

Tests are local and support actual-run playback:

```sh
ROBO_ARCH_VISUALIZE=1 uv run pytest \
  src/robo_arch/scenarios/camera_protection/tests/test_run.py -k default
ROBO_ARCH_VISUALIZE=1 uv run pytest \
  src/robo_arch/scenarios/camera_protection/tests/test_ground.py
```

The ground test removes the boxes and lowers the tool camera toward the floor,
then retreats. It records both filtered and nominal runs under `recordings/`;
ordinary sphere-pair constraints remain clear so floor intervention is isolated.

## Batched CUDA control in Isaac

The world resolves compatible controller adapters automatically. CPU and CUDA
use the same independent JaxSim nominal dynamics and shared feedback/barrier
laws; Clarabel and Moreau supply the scalar and batched QP solves. Both nominal
`joint_tracking` and `joint_pd` are supported. Tensor sensor observations remain
unsupported; mounted camera mass and geometry remain with observations disabled.

Reuse the scene/task YAML with a separate world profile instead of copying the
scenario. Install the optional solver once with
`uv sync --project third_party/isaac --locked --group cbf-gpu --group test`.
From the repository root:

```sh
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --world-config package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml \
  --batch-size 32 --no-browser
```

Add `--baseline` for the same batch without filtering. JAX always compiles the
shared dynamics; first use of a model/batch shape includes compilation. Set
`XLA_PYTHON_CLIENT_PREALLOCATE=false` when sharing the device with Isaac.
The world profile owns `num_envs`, `env_layout` and `env_spacing`. Trace stride
belongs to `task.parameters.log_every_n_steps`. Clones have translated
origins and collision isolation; the controller works in the common environment
coordinates. One failed QP stops the batch. Native PhysX effort is float32; the
filter includes rounding protection and validates the command actually applied.
Finite joint-velocity bounds enter the same QP on both CPU and GPU, keeping
filtered motion away from native velocity clamping.

This scenario supports Isaac Lab with PhysX GPU/PGS physics. Newton camera
protection is rejected until separately validated. With the imported 32 position iterations,
GPU/TGS lost low-speed joint-position increments while still reporting nonzero
velocity: at 0.001 rad/s over 20 ms, five joints remained stationary instead of
moving approximately 20 microradians. PGS measured approximately 19–20
microradians. The inconsistent TGS state caused a measured 0.21 mm clearance
violation despite accepted nominal QPs; configuration now rejects that solver
for this scenario. The Lab execution tests retain the low-speed regression.

Trace arrays have shape `[samples, environments, signals]`. Reports retain
per-environment results and aggregate worst-case clearance/error, and all must
pass. Clearance minima, residual minima and intervention maxima are accumulated
on the GPU at every control step; downsampling exported traces does not reduce
those checks. Plots show the worst logged environment and any active floor row.
The report names control and trace sample periods separately. It also checks
minimum joint-velocity slack and velocity-barrier residual at every control step;
`minimum_cbf_residual` retains its geometry-only meaning. Task evaluation
assumes one uninterrupted approach/retreat episode; the lower-level world reset
API is available to other training loops.

On an RTX 3060 Laptop GPU with the pinned Isaac Lab PhysX/PGS profile,
two-environment compiled runs at the 1 ms control period retained positive
clearance: 8.23 µm beyond the 10 mm margin in the obstacle case and 69.11 µm
beyond it in the floor approach. Both environments retreated within the configured
tolerance. The unfiltered obstacle comparison reached −57.95 mm clearance, and
the unfiltered floor comparison reached −26.17 mm. These local correctness runs
shared a heavily loaded workstation; the six-second filtered obstacle run took
699.9 s of simulation wall time, excluding setup/compilation and artifact export.
It does not provide real-time control.

The control-only benchmark includes independent dynamics, nominal control, all
128 geometry rows, 12 velocity rows, Moreau and status synchronization, excluding physics, rendering
and trace export:

```sh
uv run --project third_party/isaac --group cbf-gpu python \
  -m robo_arch.scenarios.camera_protection.benchmark
```

The benchmark records compilation/setup separately from steady-state calls.
Previous Torch-tree timings do not describe the shared JaxSim implementation;
rerun the benchmark for the selected model, batch, device and pinned environment.
It writes measured JSON and a plot under `recordings/`. GPU residency is not a
claim of 1 kHz control or hardware suitability.

Native GPU integration tests exercise two environments through both obstacle and
floor approaches, with and without filtering. Each case starts a separate Isaac
process and retains its measured trace, plot and resolved inputs:

```sh
ROBO_ARCH_NATIVE_ISAAC=1 OMNI_KIT_ACCEPT_EULA=YES \
  uv run --project third_party/isaac --group cbf-gpu --group test python -m pytest -q -s \
  src/robo_arch/scenarios/camera_protection/tests/test_gpu_run.py
```

These tests can take several minutes per case,
including first-use compilation. Lower-level native
reset and failure-cleanup tests are documented with the
[world implementation](../../core/worlds/README.md).
