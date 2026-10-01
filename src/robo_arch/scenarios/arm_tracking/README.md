# Arm tracking examples

| Run resource | Assembly | Autonomy | Observations |
|---|---|---|---|
| `scenario.yaml` | UR7e + D435 | Drake inverse dynamics | Ideal RGB-D in Drake |
| `iiwa7.yaml` | iiwa 7 + Mini45-R | Native PD + gravity | Six-axis wrench |
| `bimanual.yaml` | UR7e + nested iiwa system | Independent native PD per arm | Two named wrenches |
| `iiwa7_contact.yaml` | iiwa 7 + Mini45-R, fixed block | Native PD into an obstructed target | Wrench peak check |

All use `python -m robo_arch.scenarios.arm_tracking.run --run
package://robo_arch/scenarios/arm_tracking/<resource>`. Build/install the native
controller first with `uv run tools/dev.py native --profile drake`; use the
`isaac` profile in that separate environment. Both new systems run in Drake and
Isaac; the mixed system uses `left_arm`, `left_ft`, `right/arm` and
`right/wrist_ft`. The system definition never fixes its controller.

Execution lives in each world’s `scenario.py`; this package supplies task-specific
autonomy wiring in `drake.py` and `isaac.py`, then evaluates the returned traces.
See the [world entry points](../../core/worlds/README.md#native-entry-points).

## Configuration

YAML references inside configuration always use `package://robo_arch/...`.
CLI `--run` and `--world-config` additionally accept explicit filesystem paths.
Unknown fields and recursive system inclusion fail before construction.

A single-arm task retains `robot`, `target` and `tolerance`. A multi-arm task uses
`robots: {<instance>: {target: [...], tolerance: ...}}`; autonomy parameters use
`robots: {<instance>: {kp: [...], kd: [...]}}`. Every configured arm must be named
exactly once. Vectors follow the device's declared joint ordering, with positions
in radians, velocities in rad/s and tolerances in radians.

`joint_tracking` gains are acceleration-feedback gains in s^-2 and s^-1.
`joint_pd` gains are torque feedback in N m/rad and N m s/rad. These are separate
controllers, not interchangeable gain presets. The native controller's gravity
model includes mounted sensor inertia and applies actuator effort limits.

Task parameters can include `wrenches: {<sensor>: {min_peak_force_N: 1.0}}`.
This evaluates the norm of measured force after the first physics step; selecting
an absent or disabled wrench sensor fails early. The contact example deliberately
commands beyond the block, uses a 0.04 rad tracking tolerance and requires a force
peak above 1 N. It demonstrates contact/sensing, not force-feedback control or
safe hardware motion. Its fixture placement is a nominal scenario assumption.

Physics time steps, solvers and viewer settings belong to the selected world.
`--world` replaces the complete world configuration with native defaults;
`--world-config` selects a complete profile. `--headless` controls viewing only.
`--no-sensors` disables observations while preserving device bodies, masses and
collisions. Unsupported observations are errors, never silently dropped.

## Inspection

```sh
uv run python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --record recordings/bimanual_drake.html --no-browser
uv run python -m robo_arch.scenarios.arm_tracking.run \
  --inspect recordings/bimanual_drake.json --visualization live_and_record
```

The JSON report includes all resolved inputs, package versions, source/asset
hashes, per-arm results and a copyable inspection command. Restoring a report
uses the current installation and writes separate inspection artifacts.
`--metadata` chooses the report and default NPZ/PNG basename; `--trace` overrides
the NPZ destination. NPZ channels use `<robot>/q`, `<robot>/v`, `<robot>/effort`
and `<sensor>/wrench`, plus a common `times` array in seconds. Each array owns
its samples. Wrenches are `[Fx,Fy,Fz,Tx,Ty,Tz]` in N and N m; see the
[Mini45 convention](../../sensors/ati_mini45/README.md). Initial wrench samples
are NaN because no physics step has produced a valid reaction yet. Drake runs
with one `Simulator.AdvanceTo()` call and exports native signal logs; sample
times follow simulator steps, including viewer events and the final boundary.
Its effort channel is the plant's sampled net actuation (zero before the first
step). Isaac records observations and applied commands in its native stepping
loop. Plotting lives in this scenario, with radians and N m assumed for these
revolute arms.

Drake playback contains actual simulated geometry transforms. Use live contact
layers for changing hydroelastic surfaces; HTML does not faithfully replay
those surfaces. Isaac supports a live native Storm viewport using
`--world-config package://robo_arch/core/worlds/isaac/desktop.yaml`; it holds the
final scene and saves `<report>.viewport.png`. Keep `DISPLAY` set. Window closure
or Ctrl-C releases the viewer. RTX cameras, contact overlays and video recording
remain unsupported. Partial traces survive runtime errors. Tests print commands for inspecting the same inputs.
