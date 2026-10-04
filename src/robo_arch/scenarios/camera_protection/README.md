# Camera protection

This Drake example mounts three D435 housings on a UR7e's forearm and two wrist
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
uv run python -m robo_arch.scenarios.camera_protection.run --no-browser
uv run python -m robo_arch.scenarios.camera_protection.run --baseline --no-browser
uv run python -m robo_arch.scenarios.camera_protection.run \
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
For native PD, first run `uv run tools/dev.py native --profile drake` as described
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
checks 125 sphere pairs and three ground constraints at 1 kHz. Pair algebra is
vectorized and point queries are grouped by frame; the remaining work includes
dynamics, an effort QP when
the nominal command is unsafe, logging and physics. Wall-clock rate also depends
on other processes; increasing `target_realtime_rate` cannot accelerate this work.

The CBF uses privileged simulated state, fixed known obstacle poses and nominal
rigid-body dynamics. The 1 ms simulation samples a continuous-time barrier law;
its recorded margin is measured evidence, not a guarantee between samples or
under unmodeled contact forces. Hardware, moving obstacles, Isaac and batched GPU
execution are not implemented. Python adapters reject unsupported worlds.

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
