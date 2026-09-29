# Implementation plan: private repository to shared controller execution

This is the recorded implementation plan. Work packages become executable assignments when delegated; recording the plan does not mean they have been performed. Follow [architecture.md](architecture.md), [build_and_layout.md](build_and_layout.md), and [AGENTS.md](../AGENTS.md).

## 1. Outcome and implementation boundaries

Establish **`AlexandreAmice/robo_arch` as a private GitHub repository**, then deliver a first milestone demonstrating:

- Editable Python development through uv.
- C++23 components built by Bazel and immediately usable from Python simulations through nanobind.
- Bazel as the primary CI and testing interface.
- Independently composable scenario, autonomy, and world configurations.
- The same configured controller running in Drake and Isaac, including independent state and reset in an Isaac batch.

R0, B0, I0 and the limited G0 feasibility work have landed; retain their recorded evidence below. The device-oriented migration, single-arm Drake/Isaac execution and typed world configuration are implemented. Native Isaac viewing remains deferred. Do not repeat repository setup or discard the build baseline.

**Defaults:** Linux x86-64, Python 3.12 minimum, Python-first implementation, ordinary YAML with PyYAML/Pydantic, and local GPU execution first. Preserve the agreed architecture and existing user edits.

The local GPU has 6 GB VRAM, below the current published Isaac minimum of 16 GB. Start with a headless, camera-free compatibility experiment. An unsuccessful experiment blocks Isaac execution evidence, not unrelated implementation. Do not provision paid infrastructure or substitute another simulator automatically. [Isaac requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html)

The first milestone excludes physical robot motion, complete nut placement, learned-policy training, and native GPU inverse dynamics. Those follow through separately scoped work packages below.

## 2. Establish GitHub and the collaboration workflow

### R0 — Initialize and push the private repository

**Owner:** coordinating agent. **Dependencies:** none.

1. Preserve the current documents, including this work breakdown.
2. Add ignore rules for virtual environments, Bazel output links, caches, generated binaries, local environment overrides, browser/tool scratch output, recordings, checkpoints, and agent scratch plans. Retain source assets and dependency lockfiles.
3. Initialize Git with `main`, stage the intended files explicitly, and create the initial documentation commit.
4. Create the private repository and push:

   ```text
   gh repo create AlexandreAmice/robo_arch --private --source=. --remote=origin --push
   ```

5. Confirm repository visibility, upstream tracking, and matching local/remote commit IDs.
6. Enable GitHub Actions for subsequent CI. Do not add a public license or publish packages.

If setup already exists, inspect and resume it without replacing history or force-pushing.

**Complete when:** the current design is recoverable from private GitHub and a fresh clone contains the intended documents.

### Rules for implementation agents

- Give each work package one owner, branch, and isolated worktree. Use separate virtual environments for worktrees that build/install different native artifacts.
- Each package produces one focused PR, or a short dependent PR series where necessary.
- The coordinating agent owns shared interface changes, root dependency pins, and integration into `main`; B0 authors the initial build/pin changes.
- Device agents own their robot/sensor packages, including assets, calibration, specializations, tests and local BUILD files. System/scenario agents own their composition packages. World agents own shared world integration and specifically assigned device wrapper files; these need explicit ownership when concurrent with device work.
- Coordinate edits to shared constructors and parameter schemas explicitly; wrapper agents provide changes to the owning package.
- An agent starts when its listed dependencies are merged. Earlier exploration is allowed, but it must not invent a competing interface.
- Every handoff states implemented behavior, public interface changes, commands actually exercised, limitations, and remaining dependencies.
- Merge after relevant checks pass. Require no independent reviewer approval that would prevent a solo maintainer from merging. Apply required checks through GitHub settings where supported.

Keep work-package status in this document; avoid creating a second architecture document for each agent. The coordinator updates status to prevent competing edits to this file.

## 3. Shared interfaces to establish before broad parallel implementation

### I0 — Configuration and construction boundaries

**Owner:** architecture/interface agent. **Dependencies:** R0.

The original component, port and resolved-graph records were introduced in I0 and are superseded by native runtime assembly. Keep validated parameters and physical instance records, lazy device factories, and explicit algorithm constructor arguments. Python composes Drake Systems/Diagrams; configuration does not encode an autonomy graph or universal factory context.

Check robot/joint identity, command modes and model dimensions at construction boundaries. Keep configuration inspection independent of simulator imports. Add new records only for concrete consumers rather than predeclaring timing, reset or capability frameworks.

### L0 — Migrate existing source ownership

**Owner:** one migration agent. **Dependencies:** completed B0/I0. **Status:** implemented; integration PR pending.

Move the existing `contracts/` and `config/` packages under `src/robo_arch/core/`, keeping their tests beside them. Update imports, Bazel labels, package exports, build smoke-test references and README commands in one change. Record semantics are handled separately from the layout migration. There are no robot/sensor implementations to relocate yet. Do not scaffold empty device directories or keep a second copy of the old packages.

Retain dependency pins and the uv/Bazel workflow. Audit the Python wheel's included files and explicit Bazel data conventions so owner-local YAML/assets can be added without relying on the editable checkout. Preserve the existing native-wheel boundary; it must not collect every device SDK. Update completed-work documentation only where current import paths or commands change, preserving historical evidence.

**Complete when:** the existing focused tests/import checks pass at the new paths, including an installed Python import outside the source tree, with no numerical or configuration behavior change.

### I1 — Device and recursive robot-system descriptions

**Owner:** interface agent. **Dependencies:** L0.

Keep SDK-independent configuration for named robot, sensor and child-system instances and attachment frames. Physical configuration selects assets and placement; autonomy assembly stays in Python. Validate available measurements and accepted commands at the owning construction boundary without a parallel port schema.

Scope instance identities through the system tree using namespaced device names. Two child systems can reference the same definition while having distinct device IDs, calibration selections and state. Device multiplicity is separate from the environment batch dimension: each batched environment contains its own left/right devices.

Represent device-owned and relationship-owned calibration profiles with explicit target instances, device identities, mounting revision and affected frames. Internal attachments belong to the system; external placement belongs to the scenario. Define one effective selected value for each relationship, and check numerical applicability during physical assembly.

Update the unreleased records and their tests together, without retaining parallel legacy scenario schemas. Keep the first arm-only system valid with no sensor/gripper requirement. Use metadata-only wrist-camera and bimanual fixtures to establish recursive composition; those fixtures do not claim actual device support.

**Complete when:** declarations represent an arm-only system, two instances of a nested arm-with-camera system, task-object subsets and separate autonomy selection, without importing SDKs or sharing instance identity.

## 4. Work packages and parallel execution

### Wave A — Independent foundation work

Completed first batch, retained here as the foundation. Current continuation points are recorded in the status table below.

**B0 — Build and dependency baseline**

Own root build metadata and toolchain configuration. Establish Bzlmod, pinned C++23/Python toolchains, editable Python packaging, Ruff, Drake-derived clang-format settings, and Buildifier. Read the uv lock directly into Bazel's Python dependency repository. Establish separate core, Drake, and Isaac environment profiles. Select exact dependency revisions through compatibility checks and commit the successful matrix; dependent packages consume those pins. Do not silently relax C++23 or the Python baseline. Coordinate the Isaac profile's runtime requirements with G0; G0 owns the vendor-environment files.

**I0 — Shared interfaces**

Implement the interface package described above. It can develop alongside B0; its checks become Bazel targets once B0 lands.

**G0 — Local Isaac feasibility**

Own the pinned vendor-environment description. Check the selected Isaac runtime's Python, driver, memory and GPU requirements, then attempt a minimal headless physics example without cameras. Record exact versions and startup/resource results. Use a separate environment from base development. Stop runtime expansion on a demonstrated compatibility/resource failure; report the blocker and continue non-GPU work. Do not change system drivers or the base development environment to force the probe to pass.

### Wave B — Infrastructure using the shared baseline

**B1 — Nanobind and native development bridge**

**Depends on:** B0, L0.

Own binding build helpers, native wheel packaging, and the development helper.

Provide `native` and `run` operations, with an explicit environment profile:

- Ask Bazel to incrementally build the profile's native wheel.
- Check the active Python/native ABI combination.
- Install the resulting artifact without resolving dependencies again.
- Skip reinstalling an unchanged artifact only when the installed artifact matches.
- Launch the requested Python command in a fresh process.
- Stop on build/install failure rather than running stale native code.

Use a small numerical C++ library to prove the mechanism. Keep its Python binding thin and its C++ target independently linkable. Shared native algorithms live with their shared owner; device-specific native code stays with the device. The wheel manifest maps either location to private extensions. The source-development profile must not pull in a competing released native wheel.

**Complete when:** changing C++ changes the result observed from editable Python, ordinary Python edits require no native rebuild, and a failed native build prevents launch.

**C0 — Bazel testing and GitHub Actions**

**Depends on:** B0, L0; add native packaging coverage after B1.

Own CI workflows and shared pytest integration.

Run core tests through Bazel using declared sources, dependencies and resources. Add Drake checks after its integration exists. Reuse the same pytest sources under uv and Bazel. Include formatting checks, `uv lock --check` and Bazel `--lockfile_mode=error` checks.

Keep GPU jobs explicitly selected; a selected integration job must fail if its required runtime is unavailable. Do not count skipped GPU tests as Isaac validation. Keep remote caches and CI artifacts private.

**Complete when:** a fresh GitHub runner executes core Bazel checks without a developer `.venv`.

**S0 — Physical configuration and selection**

**Depends on:** B0, I1.

Own configuration loading and resolution under `core/config/`.

Typed, SDK-independent [world configuration](architecture.md#world-configuration-and-visualization) is implemented with complete inline/package profiles, effective settings and rejection of foreign fields. Calibration and deployment-specific tuning remain.

Implement safe YAML loading, package resource references, parameter validation, recursive physical assembly and explicit supported-world checks. Autonomy is assembled in Python using native runtime APIs; do not implement graph parsing or port inference. Reject recursive definition inclusion, missing instances/frames, conflicting attachments, incompatible calibration, missing world implementations and incompatible commands. Report supported slow execution as a warning. Load only selected device packages through the fixed package convention; world assembly calls their explicit native construction functions when needed.

Include fixtures demonstrating:

- Robot-system or device replacement while retaining scenario objects and task.
- A bimanual system reusing one child-system definition with distinct names and hardware/calibration bindings.
- A task referencing only a subset of scene objects.
- Layout replacement independently of object definitions.
- Device, assembly and scenario calibration selected at the owning scope, without duplicate effective transforms.
- Python-authored autonomy using explicit model, parameter and joint-order dependencies.

**Complete when:** both valid resolution and relevant failures are inspectable without loading Drake, ROS or Isaac.

**A0 — UR7e model and model loading**

**Depends on:** I1 and B0.

Own `robots/ur7e/` and the minimal arm-only robot-system description. Reusable model-loading utilities belong under `core/models/`.

Pin the actual UR7e description with provenance and assets inside its package. Establish named joints, frames, model parameters and actuator mappings. Put UR7e-specific calibration/model profiles there; reserve inter-device transforms for the system. Keep control-model construction distinct from simulation state. Exclude the unspecified gripper from the first arm example rather than silently substituting one.

**Complete when:** world and controller construction consume the same identifiable robot description.

### Wave C — Shared computation and world integration

**K0 — Shared tracking algorithms**

**Depends on:** B0, I1, A0.

Own shared reference-generation/controller implementations and the UR7e controller specialization. Assign both locations to this agent to avoid competing control implementations.

First provide a reusable Python construction function for position-command tracking. Then add inverse dynamics using the existing Drake controller/dynamics implementation under `core/controllers/`. UR7e model/joint selection and its tuning live under `robots/ur7e/controllers/`; system-specific presets belong to the system. Define reference conventions, force accounting and state/reset behavior once. A robot-system description advertises available interfaces without installing this controller into every autonomy stack.

World adapters must call this computation rather than reproducing controller equations. Keep Python where practical; use B1 when native batch invocation becomes necessary.

**Complete when:** controller behavior can be exercised independently of a simulator with explicit inputs and state.

**D0 — Drake world and generic assembly**

**Depends on:** S0, A0, K0.

Own generic Drake scene/Diagram assembly under `core/worlds/drake/` and explicitly assigned device `drake/` wrappers. Keep UR7e-specific construction in its package. Generic physical assembly handles nested systems; scenario or system Python owns task-specific autonomy wiring.

Construct simulation models, measurement sources and accepted command ports. Compose autonomy with native Systems and DiagramBuilder, using shared controller construction functions. Keep the assembly reusable for later real-world driver substitution, following the separation illustrated by Drake's HardwareStation approach. [HardwareStation reference](https://manipulation.mit.edu/python/station.html)

Provide a Python-authored tracking run. Record configured rates, controller identity and effective parameters.

Apply selected Drake physics settings at construction and use the standard visualization configuration for illustration, proximity, inertia and contact publication. Keep viewer lifecycle in the world package and verify contact diagnostics with explicit geometry/properties; UR7e mesh/contact assets remain A0 work. Follow the [inspection requirements](architecture.md#viewer-lifecycle-and-inspection).

**Complete when:** the declared position stack runs, followed by inverse dynamics, without per-demo wiring inside the world implementation.

**X0 — Isaac world and generic assembly**

**Depends on:** successful G0, S0, A0, K0.

Own generic Isaac scene/execution integration under `core/worlds/isaac/` and explicitly assigned device `isaac/` wrappers. Keep robot/sensor-specific logic in those packages.

Reuse controller algorithms and parameter schemas through an explicit Isaac wrapper. Start with one arm and no camera. Explicitly map joint order and command modes. Disable or account for simulator drives that would add unintended control behavior.

Use explicitly registered scalar/CPU wrappers initially. Preserve implementation identities and report transfers. Do not replace inverse dynamics with an Isaac controller for convenience.

Apply Isaac-specific physics settings and configure its native viewer/debug display independently of sensor rendering. Validate the supported API against the selected environment. Treat optional Drake geometry replay as a labeled secondary inspection path, not evidence of Isaac contact visualization. The real-world counterpart belongs to later ROS 2 integration, with transport settings and a proposed RViz 2 view.

**Complete when:** the same configured controller operates in both worlds and produces corresponding outputs for matched inputs/state.

**D0 and X0 can run in parallel** once their shared dependencies land. Neither owns a separate controller implementation.

### Wave D — Batch reuse and milestone integration

**X1 — Batched shared execution**

**Depends on:** B1, X0.

Extend Isaac execution from one environment to a small batch, starting with two. Give each robot instance within each environment independent controller state and support selective environment reset. Keep package definitions/models shareable where immutable, without sharing live contexts or hardware handles.

Use bulk CPU invocation of the same controller where needed. A native loop around existing Drake controller instances is acceptable; duplicating its control law is not. Keep tensor operations on-device where supported and report remaining transfers.

**Complete when:** resetting one environment leaves the others' controller state intact and the execution report accurately identifies CPU/scalar work.

**M0 — First milestone integration**

**Depends on:** B1, C0, S0, D0, X0, X1.

Own the short user workflow and integrated examples. Exercise the complete C++ edit → rebuild/install → Python simulation loop, plus Bazel tests and installed-package checks. Record timings separately for compilation, installation, startup and controller execution; do not promise throughput without measurements.

**Complete when:** a fresh clone can reproduce the demonstrated workflows from documented commands.

### Suggested four-agent scheduling

| Stage | Agent 1 | Agent 2 | Agent 3 | Agent 4 |
|---|---|---|---|---|
| Completed foundation | B0 | I0 | G0 limited feasibility | R0/integration |
| Layout migration | L0 | Read-only preparation | Read-only preparation | Coordination/review |
| Revised interfaces/infrastructure | I1 | B1 native bridge | C0 CI | Coordination/review |
| After I1 | S0 configuration | A0 UR7e/system | Finish B1/C0 | Coordination/review |
| Execution | K0 shared algorithms, then support | D0 Drake after K0 | X0 Isaac after K0 | Integration/review |
| Milestone | Native batch support | Drake correspondence checks | X1 batching | M0 integration |

With fewer agents, preserve dependencies and execute the same packages sequentially. With more agents, divide only packages with distinct file ownership. A scheduling row is not permission to start a task before its listed dependencies are ready.

### Following work packages

After controller reuse is established, extend the architecture through these independently owned tracks:

| Track | Deliverable | Dependencies |
|---|---|---|
| Task and scene | Assets in `objects/`; layout, task participants and evaluation in `scenarios/nut_on_pin/`; preserve the nut bore | S0 and actual part geometry |
| Hardware | UR ROS 2 driver integration and per-robot calibration in the UR7e package; shared ROS transport in shared world services | I1, A0 and selected hardware/software |
| Gripper and sensing | Robotiq package under `robots/`, camera package under `sensors/`, each with assets, wrappers, calibration and local tests | I1 and exact device selections |
| Robot systems | Mounted arm/tool/camera system, then named left/right instances for bimanual use; assembly calibration and local tests | Applicable device packages and S0 |
| Manipulation | Scenario-owned grasp/transport/insertion behavior using selected system autonomy; shared algorithms extracted where applicable | Task/scene, robot system and appropriate control |
| Learning | Policy implementation, artifact metadata and training environment using the same autonomy implementation | Batched execution and selected observation/action requirements |
| Additional worlds | Newton and MuJoCo Warp wrappers reusing shared algorithms | Existing world implementations and parameter schemas |

Task/scene and hardware work can proceed independently. Hardware-specific implementation waits for the actual gripper, sensors, dimensions and driver capabilities; agents must not invent those specifications. Learner choice and GPU optimization receive their own plans once the workload exists.

## 5. Acceptance gates

**Repository gate:** private upstream exists, initial commit is pushed, and fresh-clone instructions are correct.

**Development gate:** editable Python works without Bazel; native changes reach Python through one command; ABI mismatch and failed builds stop clearly; installed artifacts work outside the checkout.

**Configuration gate:** robot systems, their devices, task objects and layout can vary independently; nested physical systems load independently of Python-authored autonomy; reused child definitions retain distinct instances; incompatible joint/command interfaces, calibration or world support fail with instance-specific diagnostics. Packaged owner-local assets resolve without checkout-relative paths.

**Reuse gate:** Drake and Isaac select the same shared controller/model/parameters. Compare outputs using identical inputs and state with documented numerical tolerances. Closed-loop trajectories may differ between engines.

**Batch gate:** at least two environments run with independent state and selective reset. CPU/GPU transfer and scalar-execution warnings are visible. Throughput is measured, not inferred from annotations.

**CI gate:** Bazel runs the authoritative CPU/Drake checks without consuming `.venv`; selected GPU checks execute in their recorded environment. Wheel installation and resource loading receive separate coverage.

If local Isaac cannot run, publish the completed repository, development workflow and Drake results with an explicit blocked Isaac gate. Do not mark the controller-reuse milestone complete until actual Isaac execution evidence exists.

## Work-package status

Status as of 2026-09-29. Runtime, explicit assembly and world configuration are integrated through PRs #5, #7 and #6. Partial packages below retain their broader acceptance gates.

| Package | Status |
|---|---|
| R0 | Complete: private upstream, matching commits, fresh clone and Actions availability verified |
| B0 | Complete: [PR #2](https://github.com/AlexandreAmice/robo_arch/pull/2); pinned uv/Bazel baseline and separate core/Drake profiles |
| I0 | Original records landed in [PR #1](https://github.com/AlexandreAmice/robo_arch/pull/1); graph/context records superseded by native runtime assembly |
| G0 | Complete, limited feasibility: [PR #3](https://github.com/AlexandreAmice/robo_arch/pull/3); small GPU physics probe passes, vendor VRAM check fails |
| L0 | Complete: device-oriented core migration integrated in PR #5 |
| I1 | Partial: scenario selects recursive robot systems; nominal mounts and instance namespaces implemented; calibration and exported device interfaces remain |
| B1, C0 | Ready; parallel with remaining I1 work |
| S0 | Partial: strict package-resource loading, typed native world/viewer settings and effective-input inspection; calibration and batched performance diagnostics remain |
| A0 | Partial: nominal UR7e, ideal RGB-D camera, box and arm-with-camera system; collision models, gripper and calibration remain |
| K0 | Partial: shared inverse-dynamics controller executes in Drake and scalar CPU Isaac; batch execution remains |
| D0 | Partial: native plant settings, standard Meshcat geometry/contact layers, hydroelastic fixture, camera and recording; deployment reuse remains |
| X0 | Partial: CPU PGS/GPU TGS arm execution; native viewport deferred pending rendering, collision display and shutdown validation; cameras remain unsupported |
| X1, M0 | Await listed dependencies |

Foundation validation passed: 11 pytest cases, five Bazel test targets (core,
C++23, Drake dependency smoke test, contracts and configuration), nanobind library
compilation, Ruff/clang-format/Buildifier checks, dependency-lock consistency, and
wheel imports outside the checkout. See the [dependency baseline](../third_party/compatibility.md)
for pins and ABI limits. Actions is enabled; CI workflows and required CI checks
remain C0 work.

Migration review confirmed unchanged record structure after import renaming,
11 passing pytest cases, Ruff/Buildifier checks, three passing Bazel targets
(cached, with lockfile updates disabled), and isolated installed-wheel imports.
Concurrent dependency-input changes were preserved and are separate from this
reorganization.

The camera-free Isaac arm workload now runs with the shared CPU controller. The
6 GB GPU still fails the vendor VRAM check; successful physics does not establish
native rendering, camera support or batching capacity. See [Isaac evidence](../deployment/isaac/README.md).
The independent RViz launcher is unit-tested with a fake process; no installed
RViz or hardware driver was exercised. The controller-reuse milestone remains
incomplete until the native bridge, CI and independent batch/reset gates pass.

The minimal Drake run reaches its joint target and renders the fixed box. It
exercises all six source owners without claiming nut manipulation or hardware
support. The controller reads ideal joint state; the camera is observed separately.
The UR7e visual geometry is simplified and has no robot collision geometry.
Reproduction commands and current limits are in [README.md](../README.md).
