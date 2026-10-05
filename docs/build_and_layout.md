# Build and layout

Organization for the [architecture](architecture.md), based on devices and their compositions. The examples use `robots/ur7e`, `robots/iiwa7`, `sensors/realsense_d435`, `sensors/ati_mini45`, single-arm and mixed `robot_system/` compositions, `objects/box` and `scenarios/arm_tracking`; reusable loading, control and execution live under `core/`. The tree below shows the working structure; add directories only as their implementations arrive.

## Ownership and directory tree

Robot and sensor packages own their specific code, assets, calibration profiles and tests. Actuated tools are robots. `robot_system/` owns compositions of devices or other robot systems. `scenarios/` selects a robot system, object instances, placement, task and autonomy. `core/` holds algorithms and infrastructure independent of any particular robot, system or scenario. Its subpackages are named by responsibility; it is not a catch-all for code without an owner.

```text
src/robo_arch/
  robots/ur7e/
    robot.yaml                   # asset reference, frames, joint order and defaults
    assets/                      # model and provenance
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

Robot model loading belongs under `core/worlds/<world>/`: parse the asset declared in `robots/<model>/robot.yaml`, with no robot-specific imports or forwarding factories. Add ordinary robots with data alone. Simulation and controller-model construction use the same asset reference. World support follows the implemented asset formats and required behavior; a model file alone does not establish hardware support. Keep code under a device's `drake/`, `isaac/` or `real/` directory only for a concrete device-specific responsibility, such as sensor observations or a hardware driver. Autonomy implementations remain explicit and reuse shared algorithms.

Robot-system composition describes the physical assembly and its available interfaces. Autonomy composition describes computation. A system may provide convenient autonomy presets, but does not require one fixed controller or policy. Both model-based and pixel-to-command stacks can target the same robot system.

The [world configuration](architecture.md#world-configuration-and-visualization) stays inline in the scenario until reuse warrants a package-referenced file. Scenario-specific solver/viewer choices belong with that scenario; reusable parameter defaults belong to the corresponding SDK-independent world schema. Device material/contact profiles stay beside device assets, assembly-specific RViz views belong with the system, and task-specific views belong with the scenario. Third-party profiles pin incompatible vendor environments; deployment owns deployable images and process-launch material. Keep native settings adapters and viewer lifecycle in the world package; scenario runners select them instead of constructing Meshcat or RViz directly. The real-world launcher owns only RViz; ROS publishers and hardware drivers remain separate work.

## Composition, placement and calibration

A robot system recursively names device or child-system instances, declares parent-frame attachments and exposes interfaces. Actuated tools remain separate robot declarations mounted to moving frames; physical assembly and controller-model construction must preserve the combined mechanism and device-owned actuators. The current fixed-base robot records do not yet implement this target. For a bimanual system, `left` and `right` can reference the same mounted-arm definition. Their joint/frame identities, hardware IDs and mutable runtime state remain independent. Declaring a reusable assembly once must not reuse the same live driver or controller state twice.

Internal attachments and relative mounts belong to the reusable robot-system YAML. The scenario YAML selects that definition alongside autonomy settings and contains the system pose, object models/poses and task inline. Task parameters reference only participating object instances. Swapping the scenario's robot-system reference leaves its objects/task reusable, subject to interface and frame checks; valid references do not establish reachability.

Calibration belongs to the package that owns the calibrated relationship: robot unit, sensor unit, mounted assembly, or scenario fixture. Every measured profile identifies the devices, mounting arrangement and revision it applies to. Select profiles explicitly; do not infer them from folder names or silently reuse one system's calibration for both arms. Nominal assets, nominal mounting and measured corrections remain distinguishable. A pose/relationship has one effective selected value, not competing copies in device, system and scenario files.

World `scene.py` constructs physical assemblies from scene configuration. World `scenario.py` consumes the run configuration, resolves compatible autonomy implementations and invokes scenario-supplied native wiring, then initializes and executes the native runtime. Scenarios select algorithms and task references; compatibility resolution belongs to the reusable controller/world boundary, not repeated scenario backend branches. Evaluation consumes returned traces. Drake also exposes its native `Simulator` and scene directly. The existing [world construction guide](../src/robo_arch/core/worlds/README.md) records current entry points; lifecycle and backend-selection migration remain in the [implementation plan](implementation_tasks.md#architecture-acceptance-work).

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

Keep `uv.lock` authoritative for each supported environment. Bazel reads the root
lock directly through rules_python's `pip.parse(uv_lock = "//:uv.lock", ...)`,
preserving locked artifact hashes and resolution markers. Explicit target
dependencies keep core tests independent of Drake and formatting tools. uv groups
select development environments, not Bazel targets. Keep incompatible ROS/vendor
environments in independently identified profiles under `third_party/`; do not
introduce another resolver for the same environment.
[rules_python lockfile input](https://rules-python.readthedocs.io/en/latest/api/rules_python/python/extensions/pip.html)

Required local validation must check lock freshness and the environment actually
used by each requested suite. Independently locked profiles may select different
shared-package versions; test shared source against each supported profile instead
of assuming root-environment results transfer. Pin compatible Python runtimes and
native ABI settings in both workflows. Matching package versions alone does not
establish native compatibility. Isaac's Python lock does not cover Kit's runtime
extension downloads: its reproducible profile must also identify resolved
extensions and host requirements. These checks remain acceptance work described
in the [implementation plan](implementation_tasks.md#architecture-acceptance-work).

Use thin **nanobind** bindings around project C++, with explicit ownership, array layout, device and GIL behavior. Call existing pydrake APIs directly where appropriate. Exchanging bound Drake objects requires compatible Drake libraries, compiler/C++ ABI and nanobind ABI/domain/Python-ABI settings, even though both projects use nanobind. Pin that combination in build tooling. [nanobind Bazel integration](https://nanobind.readthedocs.io/en/latest/bazel.html), [interoperability requirements](https://nanobind.readthedocs.io/en/latest/faq.html#how-can-i-avoid-conflicts-with-other-projects-using-nanobind)

### API documentation

The Python API reference combines Python-source docstrings and C++ Doxygen
comments under the installed public `robo_arch` namespace. Each public symbol
has one documentation owner: Python implementations own their source docstrings;
direct C++ bindings use extracted comments; thin Python facades compose the C++
description with Python-only signature, conversion, ownership and GIL notes.
A wrapper that changes the contract owns a complete Python docstring as a
distinct API. Do not duplicate shared prose. Private extension names are not
public reference entries.

`tools/docs/` uses the pinned Clang toolchain to parse explicitly selected
declarations and Sphinx autodoc to render the combined reference. The
[catalogue](api/index.rst) covers shared configuration records/loaders, controller
parameters, numerical barriers, collision coverage, device discovery and world
settings. It lists the remaining runtime/device/scenario coverage gaps. Source
docstrings own API contracts; the [documentation index](README.md) routes readers
to tutorials, design rationale and operational guides. Extend the catalogue as
public APIs are maintained. The build fails on undocumented or duplicate entries
and unresolved imports. SDK-dependent APIs need an explicit documentation environment before
joining this SDK-independent catalogue; do not mock away missing implementations.

Each C++ owner declares a `cpp_docstrings` target and a `docstrings.json` mapping
of identifiers to qualified symbols and exact Clang signatures, so overloads
are selected explicitly. Supported comment markup is paragraphs, `@brief`,
`@details`, `@param`, `@return`/`@returns`/`@result`, `@note` and `@warning`, with
reStructuredText in plain text. Unsupported markup and unresolved selections
fail extraction. Extend the renderer with tests when another construct is needed.
The generated C++ header stays in Bazel output; the matching Python text is
committed beside its owner for `help()` without the optional extension. Binding
builds compare that copy against fresh extraction and reject stale text.

After editing the PD header's comments, refresh the generated Python copy:

```sh
bazel build //src/robo_arch/core/controllers/joint_pd:generated_docstrings
cp bazel-bin/src/robo_arch/core/controllers/joint_pd/generated/_docstrings.py src/robo_arch/core/controllers/joint_pd/_docstrings.py
```

Build the reference from isolated, installed Python/native wheels and locked
documentation dependencies (no simulator installation or launch):

```sh
uv run --no-default-groups --group docs python tools/docs/build.py
python -m http.server 8000 --directory build/docs/html
```

Open `http://localhost:8000`. For local checks, run
`bazel test //tools/docs:extract_test //tests/build:api_docs //src/robo_arch/core/controllers/joint_pd:native_test`.
The reference test blocks simulator/ROS imports while building every catalogue
page. It does not measure documentation coverage outside that explicit catalogue.
The same Python tests run under uv with the `test` and `docs` groups and the
native wheel installed. Ordinary Python docstring edits need no native rebuild.

### The C++ edit–run loop

Keep the editable `robo-arch` package separate from a Bazel-built `robo-arch-native` wheel containing private `robo_arch_native` extensions. Sources stay beside their components. Direct scenario entry points automate the bridge before runtime imports; they do not introduce another build system. Implemented commands:

```text
uv sync --locked
uv run src/robo_arch/scenarios/arm_tracking/run.py
uv run pytest path/to/test.py
uv run tools/native/install.py --profile drake            # refresh native code for IDE/notebook use
uv run src/robo_arch/scenarios/batched_reaching/run.py --backend newton --live --hold
bazel test //tests/build:core //src/robo_arch/scenarios/arm_tracking:run_test
```

`//tools/native:wheel` owns the project native package. Components own their C++
libraries and binding targets; the aggregate explicitly lists their outputs and
runtime files as Bazel inputs. Add an extension with `--extension <module> <file>`
and a runtime file with `--resource robo_arch_native/<path> <file>` in that target.
The packager generates the module inventory used to verify installed imports.
Adding a component changes these packaging dependencies, not the launcher or
installer. Bazel owns incremental rebuild decisions; the installer compares all
package payload bytes, including resources, before deciding whether to reinstall.

Each direct development launch performs these steps:

1. Resolve the runtime profile from the scenario's world selection and check interpreter/ABI compatibility. `--world`, `--world-config` and saved inspection inputs are honored. Both environments must already exist from `uv sync`.
2. Ask Bazel to incrementally build that profile's native wheel and dependencies. Bazel decides what changed; the helper maintains no separate C++ dependency graph. Changes outside the wheel's dependency graph do not require rebuilding it.
3. Install the exact resulting wheel into the active uv environment, without resolving dependencies again. Reinstall only when the artifact changed or is missing. Include required shared libraries/runtime resources with valid loader paths; copying just an extension is insufficient.
4. Start the requested Python command in a fresh process using that environment. A build/install failure stops the launch rather than running an old controller.

Scenario flags follow the Python filename. `--run` selects the scenario YAML;
`--world` or `--world-config` replaces its world selection, then named overrides
are validated before environment selection. Batched reaching accepts `--config`
as an alias for `--run`. Its `benchmark.py` runs the scaling comparison in Isaac.
Direct scripts configure Isaac's EULA/display settings as described in the
[vendor profile](../third_party/isaac/README.md).

Launch preparation lives in `core/worlds/launch.py`; native build/install tooling
stays in `tools/native/`. Preparation preserves the caller's working directory
and arguments, uses Python's safe-path option to avoid sibling modules shadowing
SDK packages, and consumes a PID-scoped handoff after re-execution. Each worktree
uses its own existing uv environments. Installed-package and Bazel execution use
their supplied native artifacts without building a checkout. Imports and callable
simulation APIs never trigger compilation. `tools/dev.py` remains a deprecated
compatibility adapter; new commands should use direct files.

The Python simulation remains editable, and both paths use the same native Bazel targets. Local wheels need no manylinux release repair on every edit. Wheel assembly/install has overhead; measure it before introducing a more complex editable native-artifact scheme. No compilation happens implicitly on import.

Ordinary `uv run` retains additional installed packages by default; exact `uv sync` can remove a development wheel. The next direct scenario launch restores it after synchronization. Keep a released native-wheel dependency out of the source-development profile so it cannot compete with the local build. The helper launches its child directly rather than syncing again. Restart notebook kernels after native changes; rebuilding cannot replace an extension already loaded into a process. [uv synchronization behavior](https://docs.astral.sh/uv/concepts/projects/sync/)

### Local validation and future release packaging

Run Bazel test suites locally against declared Python libraries, data and native
targets, without a developer checkout path or installed development wheel
supplying undeclared dependencies. Keep BUILD declarations small and
package-scoped: ordinary Python edits need no BUILD changes; new files/dependencies
must enter the declared graph. Provide one local validation entry point that
selects explicit core, Drake, vendor-numerical and native integration suites.
Vendor suites use their identified runtime environment; a source-only Bazel target
does not establish runnable SDK support. A requested suite must fail clearly if
its dependencies or runtime are unavailable. Unrequested optional suites may be
omitted, with the executed coverage reported. This validation interface remains
to be implemented. Do not add hosted workflows or required remote checks.

Prefer hermetic compiler, interpreter and package inputs through the existing
Bazel/uv mechanisms. Where host setup is necessary, provide a small script under
`tools/` for the selected profile's prerequisites, including a check-only mode.
Keep GPU drivers, display services and Apple's SDK explicit host
requirements; do not silently change drivers or compile missing SDK dependencies
through an alternative build path. Identify ROS's distribution packages or
ament/colcon underlay separately from uv packages; record its package versions
and interpreter/ABI, and keep it out of root/Isaac resolution.

Initial OS scope is Ubuntu and macOS. The proposed first matrix is deliberately
narrow; the targets below do not claim clean-machine validation:

| Host | Runtime scope | Current limit / required decision |
|---|---|---|
| Ubuntu 24.04 x86-64 | Core Python/native and Drake | Existing local baseline; clean second-machine and transferred-artifact checks remain. |
| Ubuntu 24.04 x86-64 with suitable NVIDIA GPU | Isaac and vendor numerical suites | Existing workload-specific evidence; record driver, resolved Kit extensions and viewing requirements separately. |
| Ubuntu 24.04 x86-64 | UR driver/mock/URSim | Proposed ROS 2 Jazzy underlay; exact driver, middleware and URSim image remain unselected. |
| macOS arm64, release to select | Core Python/native and Drake | Target only; choose compatible interpreter/Drake artifacts and native toolchain/SDK settings first. Isaac and ROS execution are outside this initial macOS target. |

The committed Drake 1.57.0 lock contains macOS arm64 wheels for CPython 3.13/3.14,
but the native wheel and Isaac profile currently require 3.12. Prefer a supported
stable Drake artifact compatible with the shared interpreter before introducing
a separately pinned macOS interpreter profile. That choice remains open; do not
infer macOS support from Python source compatibility or silently build Drake
from source. Current [Drake platform guidance](https://drake.mit.edu/installation.html)
is background for selecting a future compatible pin, not evidence for this lock.

Recommend one small compatibility record under `third_party/` for the selected
OS/architecture/runtime combinations, host requirements and evidence references.
It should reference existing locks/toolchain declarations rather than copy their
versions. Render the maintained support table and select local validation suites
from that same record. Available wheels establish eligibility, not successful
execution; distinguish proposed, locally exercised and clean-machine-validated
combinations. The record format and generator remain implementation work.

**Source reproduction** starts from a fresh checkout on a clean second machine,
resolves identified profiles and builds/runs without developer caches or local
paths. **Artifact transfer** installs produced wheels or a runtime image on
another supported machine and runs without rebuilding. Compare within each
declared OS/architecture/ABI combination; a Linux wheel is not a macOS artifact.
Test packaged declarations/assets outside the checkout with editable imports
excluded. The [acceptance gates](implementation_tasks.md#acceptance-gates) own
the evidence requirements; same-machine containers do not replace a second
machine. Device-dependent integration suites need explicit caching policies.

The current local CPython 3.12 Linux x86-64 wheel uses host glibc and statically
linked, hidden C++ runtime/nanobind symbols; it is not a portable manylinux release
artifact. It passes no Drake C++ objects across bindings. A future portable wheel
can use a pinned manylinux-compatible build environment and
[auditwheel](https://github.com/pypa/auditwheel) inspection; accurate tags alone
establish neither portability nor pydrake ABI compatibility. Record native ABI,
resolved vendor extensions and required GPU/driver environment with the tested
artifact. Keep SDK profiles under `third_party/` and deployable runtime images
under `deployment/`; image packaging must not introduce another native build path.

## Testability

Use pytest for Python and GoogleTest where native behavior needs direct coverage. Exercise shared algorithms without starting a world; inject required models/inputs/transport. Component tests cover meaningful numerical behavior, bindings and wrappers. Root tests cover composition and installed artifacts. Reuse the same Python tests under uv and Bazel.

Headless validation is primarily for automated tests. Every run presented for user testing or inspection must include an attached visualization of that run (such as an interactive recording, video or diagnostic plot) and a copyable command to launch its visualization locally. Choose a view that exposes the behavior being evaluated.

Failing tests must offer a simple visual inspection path using the same test case, configuration, seed and initial state, where applicable. Include the launch command and any required artifact paths in failure output; retain enough data to inspect the failure even if execution stops early. Simulation tests should support scene playback, numerical tests should expose relevant traces or plots, and configuration failures should identify the offending inputs without requiring a simulator. Keep visualization optional for automated execution and reuse the tested construction and execution code. The arm-tracking runner retains effective configuration, results/errors and an inspection command alongside Drake playback or per-arm state/effort and sensor traces. Isaac supports live Storm inspection and a final viewport PNG; RTX rendering remains experimental; RViz has process-level tests only. It provides an opt-in visual test rerun; see its [usage guide](../src/robo_arch/scenarios/arm_tracking/README.md).

Separate core and SDK-dependent suites through the local validation interface
above. Check missing-support errors, slow-implementation warnings and independent
batch reset. Numerical parity tests must execute in a profile containing their
providers; skipped Torch/solver tests provide no parity evidence. Compare shared
controller outputs for matching inputs/state, not entire trajectories across
different physics engines. Documentation work needs no tests or builds.
