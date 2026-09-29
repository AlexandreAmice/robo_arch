# Arm tracking

Run the packaged [scenario.yaml](scenario.yaml) from the repository root:

```sh
uv run python -m robo_arch.scenarios.arm_tracking.run
```

This saves `recordings/arm_tracking_drake.html` and opens scene playback in a browser.
Use **Open Controls → Animations → default** to pause or scrub time. The arm
moves only 0.05 rad from its starting pose. The model uses simplified visuals
without robot collision geometry; the ideal wrist camera has no visual geometry.

Use `--record path/to/playback.html` to choose an output, `--no-browser` to save
without opening it, or `--headless` for automated JSON-only execution.
`--headless --record path/to/playback.html` also records the run. `--world`
overrides the configured runtime; its implementation and environment must be available. Playback remains
viewable after Python exits. Failures after simulation initialization retain the
partial recording; invalid configuration is reported before a scene exists.

## Configuration

[scenario.yaml](scenario.yaml) contains all run settings:

| Setting | Meaning |
| --- | --- |
| `world`, `duration`, `time_step` | Selected runtime, end time and discrete plant step in seconds. The step is not a separate controller update period. |
| `robot_system.definition`, `robot_system.pose` | Reusable physical assembly and optional placement in world. |
| `robot_system.autonomy` | Controller selection and gains; `kp` is in s⁻² and `kd` in s⁻¹, with one positive finite value per joint. |
| `objects` | Named model instances with their world poses; these objects are fixed fixtures. |
| `task.parameters` | Robot instance, target joint angles in radians, and maximum final absolute joint error (`tolerance`, radians). |
| `sensors_enabled` | Whether to construct the assembly's sensors; defaults to true. `--no-sensors` explicitly omits them for a run. |

The separate [system.yaml](../../robot_system/ur7e_ideal_camera/system.yaml)
defines robot/sensor instances and mounts. Selecting an assembly does not fix its
controller. Model identifiers discover package-owned `DEFINITION` values in
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

## Inspecting test failures

Tests normally run headless. Rerun the tracking test with the same inputs and
visualization enabled:

```sh
ROBO_ARCH_VISUALIZE=1 uv run pytest \
  src/robo_arch/scenarios/arm_tracking/tests/test_run.py -k test_headless_tracking
```

This saves and opens `recordings/test_tracking.html`. The command is also included
in failure output. For Bazel:

```sh
bazel test //src/robo_arch/scenarios/arm_tracking:run_test \
  --test_env=ROBO_ARCH_VISUALIZE=1 --nocache_test_results
```

Open `test_tracking.html` from the test's undeclared outputs under
`bazel-testlogs/src/robo_arch/scenarios/arm_tracking/run_test/test.outputs/`
(extract first if output zipping is enabled). Recording uses the tested scene,
controller and execution; it does not run a separate demonstration.
