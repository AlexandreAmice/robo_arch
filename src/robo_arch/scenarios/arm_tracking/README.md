# Arm tracking

Run the packaged [scenario.yaml](scenario.yaml) from the repository root:

```sh
uv run --locked python -m robo_arch.scenarios.arm_tracking.run
```

This saves `recordings/arm_tracking_drake.html` and opens scene playback in a browser.
It also saves the final wrist-camera image beside the playback as a PNG.
Use **Open Controls → Animations → default** to pause or scrub time. The arm
moves only 0.05 rad from its starting pose. The model uses simplified visuals
without robot collision geometry; the ideal wrist camera has no visual geometry.

Use `--record path/to/playback.html` to choose an output, `--no-browser` to save
without opening it, or `--headless` to disable visualization while preserving
sensor observations. `--headless --record path/to/playback.html` records without
requiring a live view. Native Drake modes are `off`, `live`, `record` and
`live_and_record`; for example:

```sh
uv run --locked python -m robo_arch.scenarios.arm_tracking.run \
  --visualization live_and_record
```

Live inspection keeps the final scene open until **Close inspection** or Ctrl-C.
Drake's standard viewer provides separate illustration, proximity, inertia and
contact layers. HTML preserves transforms and force arrows; changing hydroelastic
surfaces and pressure need live inspection. The display does not add the UR7e's
missing collision geometry.

`--world drake|isaac|real` replaces the whole world configuration with native
defaults, including viewer `off`. `--world-config` accepts a complete file or
package URI. Isaac currently supports only `off`; live viewing and `--record`
are rejected. See [Isaac setup](../../../../third_party/isaac/README.md).
Real-world declarations are inspectable, but this scenario has no hardware runner.

The CLI saves a JSON report with full effective inputs, configuration hashes,
package versions, result/error and a copyable inspection command. Use `--metadata`
to select its destination. `--inspect <report.json>` restores the resolved inputs;
viewer overrides affect only inspection. Code and assets are not snapshotted:
recorded hashes/versions identify the original environment. Drake runtime failures
retain partial playback where available; `--trace output.npz` retains measured
Isaac positions. Isaac inspection reruns the resolved inputs headlessly; native
viewing is deferred. Invalid configuration fails before
a scene is constructed.

## Configuration

[scenario.yaml](scenario.yaml) contains all run settings:

| Setting | Meaning |
| --- | --- |
| `world.type`, `world.physics`, `world.visualization` | Native world settings and viewer; world may instead be a complete package URI profile. See the [world schemas](../../core/config/worlds.py). |
| `duration`, `world.physics.time_step` | Scenario end time and discrete physics step in seconds; independent of controller/display intent. |
| `world.target_realtime_rate` | Drake wall-clock pacing, independent of viewer mode; zero runs unpaced. |
| `robot_system.definition`, `robot_system.pose` | Reusable physical assembly and optional placement in world. |
| `robot_system.autonomy` | Controller selection and gains; `kp` is in s⁻² and `kd` in s⁻¹, with one positive finite value per joint. |
| `objects` | Named model instances with their world poses; these objects are fixed fixtures. |
| `task.parameters` | Robot instance, target joint angles in radians, and maximum final absolute joint error (`tolerance`, radians). |
| `sensors_enabled` | Whether to construct the assembly's sensors; defaults to true. `--no-sensors` explicitly omits them for a run. |

The separate [system.yaml](../../robot_system/ur7e_ideal_camera/system.yaml)
defines robot/sensor instances and mounts. Selecting an assembly does not fix its
controller. Model identifiers select typed `describe()` functions in
`robo_arch.<category>.<model>.definition`; no device list lives in the scenario.
YAML cannot specify Python import paths. Native controller connections remain in
Python: the controller wires its robot and exposes a desired-state port; the
scenario supplies a constant target with zero desired velocity.

Every pose uses `translation: [x, y, z]` in metres and fixed-axis
`rpy: [roll, pitch, yaw]` in radians, defaulting to zero. Sensor poses are relative
to their `parent`, such as `arm/tool0`. Mounts are nominal, not measured calibration.
Initial positions, targets and gains follow `JOINT_NAMES` in the
[robot definition](../../robots/ur7e/definition.py); omitted initial positions use
`DEFAULT_POSITIONS`. Initial positions and targets must respect joint limits.
The [ideal camera](../../sensors/ideal_camera/definition.py) uses optical +z forward,
+x right and +y down. It has no noise or latency and does not guide this controller.

All YAML references use `package://robo_arch/...`; moving the scenario file or
changing the working directory does not change resolution. `--run` accepts a
scenario file or package URI. Resources must be packaged and declared in Bazel
data. Duplicate keys, unknown fields, invalid references and recursive inclusion
are rejected. Nested systems use `systems.<name>.definition` and an optional
`pose`; a child `left` namespaces `arm` as `left/arm`. The loader supports nested
assemblies; this tracking task currently requires one fixed-base arm.

## How the packaged example is assembled

The shared types are together in
[`core/config/declarations.py`](../../core/config/declarations.py).
[`load_run()`](../../core/config/loading.py) reads the scenario and its referenced
system into a `RunConfiguration`. Its `robot_system` retains the arm and camera,
local names, and relative mounts; nested child systems remain explicit.

For the packaged UR7e scenario:

1. [`resolve_devices()`](../../core/worlds/assembly.py) resolves `arm`, `camera`,
   and the camera parent `arm/tool0`. A child named `left` would produce
   `left/arm`, `left/camera`, and `left/arm/tool0`.
2. [`load_definitions()`](../../core/worlds/devices.py) calls `describe()` in
   [`robots/ur7e/definition.py`](../../robots/ur7e/definition.py),
   [`sensors/ideal_camera/definition.py`](../../sensors/ideal_camera/definition.py),
   and [`objects/box/definition.py`](../../objects/box/definition.py). These return
   model metadata and supported worlds without importing either simulator.
3. [`build_scene()`](../../core/worlds/drake/scene.py) imports the selected world
   adapters. It calls the UR7e's `add_to_plant(plant, name="arm")`, places its base,
   adds the box, and calls the camera's `add_to_builder()` with the tool frame and
   wrist mount. Each physical instance gets its own runtime state.
4. [`build_simulation()`](drake.py) directly calls
   [`joint_tracking.drake.connect()`](../../core/controllers/joint_tracking/drake.py)
   and connects the task's desired joint state to its returned input port.
5. [`run_scenario()`](run.py) advances the simulator and evaluates tracking error.

Controller selection is explicit in the scenario. This example accepts only
`joint_tracking`; another name raises an error before construction. In Isaac,
`_run_isaac()` calls the same controller's `make_policy()` CPU wrapper and warns
about the per-step state/command transfers. The robot adapter uses `add_to_stage()`.
The ideal camera has no Isaac adapter, so that run requires `--no-sensors`.
There are no implementation dictionaries or configurable Python function names.

## Inspecting test failures

Tests normally run headless. Rerun the tracking test with the same inputs and
visualization enabled:

```sh
ROBO_ARCH_VISUALIZE=1 uv run --locked pytest \
  src/robo_arch/scenarios/arm_tracking/tests/test_run.py -k test_headless_tracking
```

This saves and opens `recordings/test_tracking.html`, with effective input metadata beside it. The command is also included
in failure output. For Bazel:

```sh
bazel test //src/robo_arch/scenarios/arm_tracking:run_test \
  --test_env=ROBO_ARCH_VISUALIZE=1 --nocache_test_results
```

Open `test_tracking.html` from the test's undeclared outputs under
`bazel-testlogs/src/robo_arch/scenarios/arm_tracking/run_test/test.outputs/`
(extract first if output zipping is enabled). Recording uses the tested scene,
controller and execution; it does not run a separate demonstration.
