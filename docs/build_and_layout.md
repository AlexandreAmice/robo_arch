# Build and layout

Organization for the [architecture](architecture.md), based on devices and their compositions. The examples use `robots/ur7e`, `robots/iiwa7`, `sensors/realsense_d435`, `sensors/ati_mini45`, single-arm and mixed `robot_system/` compositions, `objects/box` and `scenarios/arm_tracking`; reusable loading, control and execution live under `core/`. The tree below shows the working structure; add directories only as their implementations arrive.

## Ownership and directory tree

Robot and sensor packages own their specific code, assets, calibration profiles and tests. Actuated tools are robots. `robot_system/` owns compositions of devices or other robot systems. `scenarios/` selects a robot system, object instances, placement, task and autonomy. `core/` holds algorithms and infrastructure independent of any particular robot, system or scenario. Its subpackages are named by responsibility; it is not a catch-all for code without an owner.

```text
src/robo_arch/
  robots/ur7e/
    definition.py                # describe(), frames and joint order
    assets/                      # model and provenance
    drake/                       # device adaptation for this world
    tests/
  sensors/realsense_d435/
    definition.py                # parameters and supported factories
    drake/
    tests/
  robot_system/ur7e_d435/
    system.yaml                  # reusable devices and nominal mounts
  objects/box/
    definition.py
    model.sdf
  scenarios/arm_tracking/
    scenario.yaml                # world, system + autonomy, objects/poses and task
    evaluation.py
    drake.py                     # task references connected to reusable autonomy
    isaac.py                     # task references for native effort callbacks
    run.py                       # CLI, world dispatch, evaluation and reports
    tests/
  core/
    config/                      # validated configuration and physical instances
    controllers/joint_tracking/  # shared control and native world adapters
    worlds/                      # device discovery and world assembly
      drake/                     # config.py, scene.py, scenario.py, visualization.py
      isaac/                     # same responsibilities; native conversion helpers
      real/                      # same entry points; hardware execution unsupported
```

Add device-specific IK, controllers, calibration and other world implementations beside their owner when needed. Actuated tools such as Robotiq belong under `robots/`; mounting calibration and coordinated autonomy belong with the robot system. Scenario fixture calibration stays with the scenario. Tests and BUILD targets remain local to their package.

Root build files, `tools/`, `third_party/`, `deployment/` and cross-package
`tests/` retain their roles. The old top-level `assets/` and `configs/` trees are
replaced by resources beside their owners. Independently locked vendor dependency
profiles belong under `third_party/`; deployable runtime images and process-launch
material belong under `deployment/`. Neither is a second copy of device drivers
or calibration. `plans/` remains ignored scratch space.

## What belongs where

| Responsibility | Owner |
|---|---|
| Robot-independent control and world adapters | `core/controllers/` (currently `joint_tracking/`) |
| UR7e model selection, joint mapping, tuning and controller/IK specializations | `robots/ur7e/` |
| Gripper protocol and actuation behavior | Selected gripper package under `robots/` |
| Camera driver, sensor model and per-unit intrinsics | Selected package under `sensors/` |
| Tool/camera mounting, assembly tuning, bimanual coordination | Owning `robot_system/` package |
| Nut placement goal, object selection, task-specific behavior and scoring | `scenarios/nut_on_pin/` |
| Generic ROS transport, configuration loading, world assembly and training support | Appropriate subpackage of `core/` |
| SDK-independent world/visualizer configuration records | `core/worlds/<world>/config.py` |
| Native runtime settings, viewer construction, publication and replay | `core/worlds/<world>/` |

Put an implementation at the narrowest scope where its assumptions hold. A UR7e controller can construct a shared inverse-dynamics implementation with its model and gains. Do not copy the equations into each robot, or force genuinely device-specific behavior into a generic interface. Assembly- or task-specific tuning stays with that assembly or scenario. `core/` contains named responsibilities, not an unstructured utility collection.

Use explicit supported-world implementations. Device-specific wrappers live in that device's `drake/`, `isaac/` or `real/` directory and call shared code where appropriate. Device adapters can call shared functions without duplicating algorithms. Generic world assembly lives under `core/worlds/`; lookup calls `describe()` in the selected `robo_arch.<category>.<model>.definition` packages. World assembly imports their world modules when needed and calls explicit native construction functions. No parallel device implementations belong in the generic world directories.

Robot-system composition describes the physical assembly and its available interfaces. Autonomy composition describes computation. A system may provide convenient autonomy presets, but does not require one fixed controller or policy. Both model-based and pixel-to-command stacks can target the same robot system.

The [world configuration](architecture.md#world-configuration-and-visualization) stays inline in the scenario until reuse warrants a package-referenced file. Scenario-specific solver/viewer choices belong with that scenario; reusable parameter defaults belong to the corresponding SDK-independent world schema. Device material/contact profiles stay beside device assets, assembly-specific RViz views belong with the system, and task-specific views belong with the scenario. Third-party profiles pin incompatible vendor environments; deployment owns deployable images and process-launch material. Keep native settings adapters and viewer lifecycle in the world package; scenario runners select them instead of constructing Meshcat or RViz directly. The real-world launcher owns only RViz; ROS publishers and hardware drivers remain separate work.

## Composition, placement and calibration

A robot system recursively names device or child-system instances, declares attachments and exposes interfaces. For a bimanual system, `left` and `right` can reference the same UR7e-with-camera definition. Their joint/frame identities, hardware IDs and mutable runtime state remain independent. Declaring a reusable assembly once must not reuse the same live driver or controller state twice.

Internal attachments and relative mounts belong to the reusable robot-system YAML. The scenario YAML selects that definition alongside autonomy settings and contains the system pose, object models/poses and task inline. Task parameters reference only participating object instances. Swapping the scenario's robot-system reference leaves its objects/task reusable, subject to interface and frame checks; valid references do not establish reachability.

Calibration belongs to the package that owns the calibrated relationship: robot unit, sensor unit, mounted assembly, or scenario fixture. Every measured profile identifies the devices, mounting arrangement and revision it applies to. Select profiles explicitly; do not infer them from folder names or silently reuse one system's calibration for both arms. Nominal assets, nominal mounting and measured corrections remain distinguishable. A pose/relationship has one effective selected value, not competing copies in device, system and scenario files.

World `scene.py` constructs physical assemblies from scene configuration. World `scenario.py` consumes the run configuration and invokes scenario-supplied autonomy wiring, then initializes and executes the native runtime. Scenario wiring calls the selected controller and supplies task references; evaluation consumes returned traces. Drake also exposes its native `Simulator` and scene directly. See the [world construction guide](../src/robo_arch/core/worlds/README.md) for entry points and extension steps.

## Dependencies, resources and tests

Device packages depend on shared code. Systems reference devices or child systems; scenarios reference systems and object assets. Shared algorithms and world assembly must not hard-code concrete device or scenario imports; discovery follows the fixed package convention for selected models. Cross-device algorithms belong to the owning system or shared code rather than introducing circular device imports.

Keep C++ implementation, nanobind bindings and Python facades beside the code they expose. Public C++ libraries stay independently linkable. Each owner has narrow BUILD targets for declarations, resources and world implementations so importing a description does not load SDKs. Use explicit typed dependencies and native runtime interfaces; add shared records only for concrete consumers.

The Python wheel includes the `src/robo_arch/` package tree. Keep small YAML/model resources under their owning package and declare them in the consuming Bazel target's `data` (or an explicit resource target); a Python dependency alone does not include undeclared data. Use `package://robo_arch/...` references for YAML configuration resources; do not depend on the working directory or editable checkout. Large assets use versioned external references. Native wheels must not sweep in every simulator's assets or dependencies.

Put tests beside the device, system, scenario or shared algorithm they exercise. Root tests cover cross-package integration and installed artifacts. Test recursive systems with two named instances, distinct calibration and independent state. Documentation-only updates require no tests or builds.

## Why `src/robo_arch`?

The checkout name is arbitrary; the inner `robo_arch` supplies the Python namespace, and `src` separates importable source from repository tooling. This supports the editable uv workflow while helping expose packaging mistakes. Bazel does not require the extra level. The new organization changes ownership within that package, not the Python packaging strategy. [Python packaging guidance](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)

## Build and Python workflow

Pinned dependencies and compatibility limits are recorded in the
[dependency notes](../third_party/compatibility.md); working development
commands are in [README.md](../README.md). A minimal Drake runner is implemented;
the native bridge below is implemented for the PD controller. Validation stays local; formal CI is not planned. World support is tracked
in the [implementation plan](implementation_tasks.md).

Use Bzlmod, committed module lockfiles, Bazelisk, explicit rule loads, narrow targets and pinned C++23/Python toolchains. Select a modern Bazel release compatible with the chosen Drake revision. Learn from Drake and `../gcs_solver_project`, but do not inherit old pins, host paths or their whole build framework. Use small symbolic macros where helpful. [Bazel modules](https://bazel.build/external/module), [symbolic macros](https://bazel.build/extending/macros)

**uv is the everyday Python interface; Bazel is the native build and independent local test interface.** Use an editable Python package in `.venv` for scripts, pytest, notebooks and IDE debugging. Editing Python requires no Bazel invocation. Bazel tests use the same source and pytest cases through declared targets, with their own pinned interpreter, dependencies and runfiles; they do not consume `.venv`.

Keep `uv.lock` authoritative for each supported environment. Bazel reads the root lock directly through rules_python's `pip.parse(uv_lock = "//:uv.lock", ...)`, preserving locked artifact hashes and resolution markers. The shared `@python_deps` repository exposes locked packages; explicit target dependencies keep core tests independent of Drake and formatting tools. uv dependency groups select development environments, not Bazel targets. Local checks verify that the uv and Bzlmod locks are current. Pin compatible Python runtimes and native ABI settings in both workflows. Shared package versions alone do not establish native compatibility. Keep incompatible ROS/vendor environments in independently locked profiles under `third_party/`. [rules_python lockfile input](https://rules-python.readthedocs.io/en/latest/api/rules_python/python/extensions/pip.html)

Use thin **nanobind** bindings around project C++, with explicit ownership, array layout, device and GIL behavior. Call existing pydrake APIs directly where appropriate. Exchanging bound Drake objects requires compatible Drake libraries, compiler/C++ ABI and nanobind ABI/domain/Python-ABI settings, even though both projects use nanobind. Pin that combination in build tooling. [nanobind Bazel integration](https://nanobind.readthedocs.io/en/latest/bazel.html), [interoperability requirements](https://nanobind.readthedocs.io/en/latest/faq.html#how-can-i-avoid-conflicts-with-other-projects-using-nanobind)

### The C++ edit–run loop

Keep the editable `robo-arch` package separate from a Bazel-built `robo-arch-native` wheel containing private `robo_arch_native` extensions. Sources stay beside their components. Use a small development helper to automate the bridge; it is not another compiler/build system. Implemented commands:

```text
uv sync --locked
uv run python -m robo_arch.scenarios.arm_tracking.run # ordinary Python work
uv run pytest path/to/test.py
uv run tools/dev.py native --profile drake            # refresh native code for IDE/notebook use
uv run tools/dev.py run --profile drake -- python -m robo_arch.scenarios.arm_tracking.run
bazel test //tests/build:core //src/robo_arch/scenarios/arm_tracking:run_test
```

The combined development command performs these steps:

1. Select a declared build/environment profile and check interpreter/ABI compatibility.
2. Ask Bazel to incrementally build that profile's native wheel and dependencies. Bazel decides what changed; the helper maintains no separate C++ dependency graph.
3. Install the exact resulting wheel into the active uv environment, without resolving dependencies again. Reinstall only when the artifact changed or is missing. Include required shared libraries/runtime resources with valid loader paths; copying just an extension is insufficient.
4. Start the requested Python command in a fresh process using that environment. A build/install failure stops the launch rather than running an old controller.

The Python simulation remains editable, and both paths use the same native Bazel targets. Local wheels need no manylinux release repair on every edit. Wheel assembly/install has overhead; measure it before introducing a more complex editable native-artifact scheme. No compilation happens implicitly on import.

Ordinary `uv run` retains additional installed packages by default; exact `uv sync` can remove a development wheel. The native helper restores it after synchronization. Keep a released native-wheel dependency out of the source-development profile so it cannot compete with the local build. The helper launches its child directly rather than syncing again. Restart notebook kernels after native changes; rebuilding cannot replace an extension already loaded into a process. [uv synchronization behavior](https://docs.astral.sh/uv/concepts/projects/sync/)

### Local validation and future release packaging

Run Bazel test suites locally against declared Python libraries, data and native targets, without a developer checkout path or installed development wheel supplying undeclared dependencies. Also exercise installed wheels outside the source tree to cover packaging. Do not add hosted workflows or required remote checks. Keep BUILD declarations small and package-scoped: ordinary Python edits need no BUILD changes; new files/dependencies must enter the declared graph. One modest pytest rule/helper should suffice.

Pin compiler/runtime inputs and execution environments. Core and Drake tests target hermetic execution; GPU/Isaac and hardware integrations need explicit worker/container/driver requirements and suitable test caching policies. Invoking those through Bazel does not make external devices hermetic. ROS dependencies may retain their supported ament/colcon build, supplied as an identified underlay.

A future portable release can build Linux wheels in a pinned manylinux-compatible environment and inspect dependencies with auditwheel. The current local CPython 3.12 wheel uses host glibc and statically linked, hidden C++ runtime/nanobind symbols; it passes no Drake C++ objects across bindings. Accurate wheel tags do not establish pydrake ABI compatibility. Simulator SDKs stay in their third-party dependency profiles, while GPU drivers remain host runtime requirements. Exact release pins and wheel ABI choices belong to the first implementation task. [auditwheel](https://github.com/pypa/auditwheel)

## Testability

Use pytest for Python and GoogleTest where native behavior needs direct coverage. Exercise shared algorithms without starting a world; inject required models/inputs/transport. Component tests cover meaningful numerical behavior, bindings and wrappers. Root tests cover composition and installed artifacts. Reuse the same Python tests under uv and Bazel.

Headless validation is primarily for automated tests. Every run presented for user testing or inspection must include an attached visualization of that run (such as an interactive recording, video or diagnostic plot) and a copyable command to launch its visualization locally. Choose a view that exposes the behavior being evaluated.

Failing tests must offer a simple visual inspection path using the same test case, configuration, seed and initial state, where applicable. Include the launch command and any required artifact paths in failure output; retain enough data to inspect the failure even if execution stops early. Simulation tests should support scene playback, numerical tests should expose relevant traces or plots, and configuration failures should identify the offending inputs without requiring a simulator. Keep visualization optional for automated execution and reuse the tested construction and execution code. The arm-tracking runner retains effective configuration, results/errors and an inspection command alongside Drake playback or per-arm state/effort and sensor traces. Isaac supports live Storm inspection and a final viewport PNG; RTX rendering remains experimental; RViz has process-level tests only. It provides an opt-in visual test rerun; see its [usage guide](../src/robo_arch/scenarios/arm_tracking/README.md).

Separate core and SDK-dependent suites. Check missing-support errors, slow-implementation warnings and independent batch reset. Compare shared controller outputs for matching inputs/state, not entire trajectories across different physics engines. Documentation work needs no tests or builds.
