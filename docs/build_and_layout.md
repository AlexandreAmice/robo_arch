# Build and layout

Proposed organization for the [architecture](architecture.md). Create directories as they acquire content; this is not a scaffolding request.

## Map files to the design

Group components into useful folders such as `controllers/` or `planners/`, without making those categories an exhaustive type system. Every component has its own directory; one that spans categories can have a directly named directory under `components/` or a new appropriate grouping. No implementation needs to become a loose top-level file. Scenario files compose independently selected robots, sensors, objects, tasks and layouts.

```text
robo_arch/                              # checkout; its name is arbitrary
  AGENTS.md
  README.md
  docs/
    architecture.md
    build_and_layout.md
    implementation_tasks.md
  src/
    robo_arch/                          # Python import package; shared C++ lives here too
      __init__.py
      config/                           # loading, references and compatibility checks
      contracts/                        # shared port/value semantics, no SDK imports
      models/                           # model loading and conversion code
      components/
        controllers/
          inverse_dynamics/
            __init__.py
            definition.py               # ports, parameters, supported implementations
            shared.py                   # algorithm or facade over native code
            inverse_dynamics.h          # optional native implementation
            inverse_dynamics.cc
            bindings.cc                 # optional nanobind module
            drake.py                    # world-specific adaptation
            real.py
            isaac.py
            BUILD.bazel
            tests/
      tasks/                            # task evaluation code, when needed
      worlds/
        drake/
        real/ros2/
        isaac/
        newton/
        mujoco_warp/
      learning/                         # training and policy/dataset support
      observability/                    # visualization and recording
      launch/                           # run/train entry points
  configs/
    robots/
      ur7e_robotiq.yaml                  # model/tool selection and named frames
    sensors/
      wrist_rgbd.yaml                    # illustrative sensor package, model still open
    scenarios/
      nut_on_pin/
        ur7e.yaml                       # selects robot, sensors, objects, task, layout
        objects.yaml                    # named nuts, pin, table and other scene objects
        task.yaml                       # goal and references to participating objects
        layouts/
          bench.yaml                    # robot/object poses and sensor attachments
    autonomy/
      trajectory_tracking.yaml          # reusable composition
      nut_on_pin_ur7e/
        stack.yaml                      # configured autonomy for this setup
        gains.yaml
    installations/
      lab_ur7e/
        setup.yaml                      # device IDs, mounting setup, calibration selection
        calibration/                    # measured robot/sensor/tool/frame relationships
    runs/
      nut_on_pin/
        drake.yaml                      # selects scenario + autonomy + world settings
        real.yaml
        isaac.yaml
  assets/
    robots/ur7e/                        # model, meshes, intrinsic defaults, provenance
    tools/robotiq/
    sensors/
    objects/nut/
    fixtures/pin/
  deployment/                           # ROS launch and pinned vendor environments
  tests/                                # cross-component and installed-package checks
  tools/                                # small build, packaging and development helpers
  third_party/                          # dependency metadata, patches, generated pip locks
  plans/                                # ignored agent work plans
  MODULE.bazel
  MODULE.bazel.lock
  .bazelversion
  .bazelrc
  pyproject.toml
  uv.lock
```

`inverse_dynamics` accepts a model, joint selection and controller parameters. Its source is reusable; `autonomy/nut_on_pin_ur7e/gains.yaml` is tuning for a particular setup. A replacement robot can reuse the objects and task while selecting different controller parameters or autonomy.

`scenarios/nut_on_pin/ur7e.yaml` references the independently reusable descriptions. A second scenario can select another robot or sensor package while referencing the same `objects.yaml` and `task.yaml`. Layout can also be reused where frame bindings fit, or replaced without copying the task. Descriptions may reference files in other scenario directories; move them to a shared directory when that makes their use clearer.

Illustrative contents of that scenario file:

```yaml
robot: ../../robots/ur7e_robotiq.yaml
sensors: ../../sensors/wrist_rgbd.yaml
objects: objects.yaml
task: task.yaml
layout: layouts/bench.yaml
```

Objects declare which instances exist. The task references its participants, such as `nut_1` and `pin`, without claiming the whole scene. Layout supplies poses relative to named frames and attachment relationships, including robot bases and sensor mounts. Intrinsic link geometry stays in the model; measured installation calibration can explicitly replace nominal placement. Missing instances, frames or conflicting attachments are configuration errors. Valid references do not prove reachability.

Asset bundles own reusable physical descriptions and nominal defaults. Measured calibration belongs to the installation configuration, identified by the relevant robot, sensor serial numbers, tool and mounting arrangement. It can apply to one device or relationships across the whole setup; changing a mount may invalidate it even when the same devices remain. Scenarios/world configurations select the applicable installation calibration explicitly, including simulation runs that need it. Keep nominal layout, measured calibration and deliberately randomized simulated geometry distinguishable.

Preserve relative references when packaging configs/assets; resolve them from the declaring file, never the current working directory. Large meshes, recordings and checkpoints live outside Git with versioned references.

A task description is not an autonomy component. If its success/failure evaluation needs code, that belongs under `tasks/`. A controller or policy that attempts the task belongs under `components/`, named for its actual implementation. Folder placement does not determine stack compatibility—declared requirements do.

## Why `src/robo_arch`?

The three names serve different purposes: the outer `robo_arch/` is the checkout, `src/` separates importable source from repository material, and the inner `robo_arch/` provides imports such as `robo_arch.components`. The checkout name has no role in Python package identity.

A conventional `src` layout helps avoid accidentally importing checkout files instead of the installed package. It needs an installation step, which our editable uv workflow supplies. This matters when developing against native wheels and vendor environments. [Python packaging guidance](https://packaging.python.org/en/latest/discussions/src-layout-vs-flat-layout/)

A root-level `robo_arch/` package would also work and save one directory level. I recommend keeping `src/robo_arch` for predictable packaging; Bazel does not require it. Placing C++ beside its Python component is our organizational choice, not a Python requirement. C++ remains independently linkable.

## Components and dependencies

Keep declarations, shared code, bindings, wrappers and component-owned tests together. Omit unused files: Python components need no C++, and worlds sharing a factory need no duplicate wrappers. `definition.py` records an explicit implementation mapping; filenames alone do not establish support.

The C++ library depends on its numerical libraries, the binding depends on that library, and world wrappers depend on the shared implementation plus necessary SDK services. Declarations load without SDKs. `worlds/isaac/` owns the simulator session and shared scene/I/O services; `components/.../isaac.py` adapts that component. Neither contains a second copy of the controller.

## Build and Python workflow

B0's selected pins and measured compatibility are recorded in the
[dependency baseline](../third_party/compatibility.md); working development
commands are in [README.md](../README.md). The native bridge, CI and world
integration described below remain later work packages.

Use Bzlmod, committed module lockfiles, Bazelisk, explicit rule loads, narrow targets and pinned C++23/Python toolchains. Select a modern Bazel release compatible with the chosen Drake revision. Learn from Drake and `../gcs_solver_project`, but do not inherit old pins, host paths or their whole build framework. Use small symbolic macros where helpful. [Bazel modules](https://bazel.build/external/module), [symbolic macros](https://bazel.build/extending/macros)

**uv is the everyday Python interface; Bazel is the native build and primary CI/test interface.** Use an editable Python package in `.venv` for scripts, pytest, notebooks and IDE debugging. Editing Python requires no Bazel invocation. Bazel tests use the same source and pytest cases through declared targets, with their own pinned interpreter, dependencies and runfiles; they do not consume `.venv`.

Keep `uv.lock` authoritative for each supported environment and generate Bazel requirements inputs from selected dependency groups, including hashes and platform markers. CI checks that exports are current. Pin compatible Python runtimes and native ABI settings in both workflows. Shared package versions alone do not establish native compatibility. Keep incompatible ROS/vendor environments separately pinned under `deployment/`. [uv/Bazel integration](https://docs.astral.sh/uv/guides/integration/bazel/)

Use thin **nanobind** bindings around project C++, with explicit ownership, array layout, device and GIL behavior. Call existing pydrake APIs directly where appropriate. Exchanging bound Drake objects requires compatible Drake libraries, compiler/C++ ABI and nanobind ABI/domain/Python-ABI settings, even though both projects use nanobind. Pin that combination in build tooling. [nanobind Bazel integration](https://nanobind.readthedocs.io/en/latest/bazel.html), [interoperability requirements](https://nanobind.readthedocs.io/en/latest/faq.html#how-can-i-avoid-conflicts-with-other-projects-using-nanobind)

### The C++ edit–run loop

Keep the editable `robo-arch` package separate from a Bazel-built `robo-arch-native` wheel containing private `robo_arch_native` extensions. Sources stay beside their components. Use a small development helper to automate the bridge; it is not another compiler/build system. Illustrative commands, not implemented CLI commitments:

```text
uv sync --locked
uv run python -m robo_arch.launch ...                 # ordinary Python work
uv run pytest path/to/test.py
uv run tools/dev.py native --profile drake            # refresh native code for IDE/notebook use
uv run tools/dev.py run --profile drake -- python -m robo_arch.launch ...
bazel test //tests:core //tests:drake                  # authoritative test suites
```

The combined development command performs these steps:

1. Select a declared build/environment profile and check interpreter/ABI compatibility.
2. Ask Bazel to incrementally build that profile's native wheel and dependencies. Bazel decides what changed; the helper maintains no separate C++ dependency graph.
3. Install the exact resulting wheel into the active uv environment, without resolving dependencies again. Reinstall only when the artifact changed or is missing. Include required shared libraries/runtime resources with valid loader paths; copying just an extension is insufficient.
4. Start the requested Python command in a fresh process using that environment. A build/install failure stops the launch rather than running an old controller.

The Python simulation remains editable, and both paths use the same native Bazel targets. Local wheels need no manylinux release repair on every edit. Wheel assembly/install has overhead; measure it before introducing a more complex editable native-artifact scheme. No compilation happens implicitly on import.

Ordinary `uv run` retains additional installed packages by default; exact `uv sync` can remove a development wheel. The native helper restores it after synchronization. Keep a released native-wheel dependency out of the source-development profile so it cannot compete with the local build. The helper launches its child directly rather than syncing again. Restart notebook kernels after native changes; rebuilding cannot replace an extension already loaded into a process. [uv synchronization behavior](https://docs.astral.sh/uv/concepts/projects/sync/)

### CI and release

CI runs Bazel test suites against declared Python libraries, data and native targets, without a developer checkout path or installed development wheel supplying undeclared dependencies. Also exercise the installed wheels outside the source tree to cover packaging. Keep BUILD declarations small and package-scoped: ordinary Python edits need no BUILD changes; new files/dependencies must enter the declared graph. One modest pytest rule/helper should suffice.

Pin compiler/runtime inputs and execution environments. Core and Drake tests target hermetic execution; GPU/Isaac and hardware integrations need explicit worker/container/driver requirements and suitable test caching policies. Invoking those through Bazel does not make external devices hermetic. ROS dependencies may retain their supported ament/colcon build, supplied as an identified underlay.

Build Linux native wheels in a pinned manylinux-compatible environment and inspect their dependencies with auditwheel. Accurate wheel tags do not establish pydrake ABI compatibility. Simulator SDKs and GPU drivers stay in the deployment environment. Exact release pins and wheel ABI choices belong to the first implementation task. [auditwheel](https://github.com/pypa/auditwheel)

## Testability

Use pytest for Python and GoogleTest where native behavior needs direct coverage. Exercise shared algorithms without starting a world; inject required models/inputs/transport. Component tests cover meaningful numerical behavior, bindings and wrappers. Root tests cover composition and installed artifacts. Reuse the same Python tests under uv and Bazel.

Separate core and SDK-dependent suites. Check missing-support errors, slow-implementation warnings and independent batch reset. Compare shared controller outputs for matching inputs/state, not entire trajectories across different physics engines. Documentation work needs no tests or builds.
