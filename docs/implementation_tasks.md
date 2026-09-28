# Implementation plan: private repository to shared controller execution

This is the recorded implementation plan. Work packages become executable assignments when delegated; recording the plan does not mean they have been performed. Follow [architecture.md](architecture.md), [build_and_layout.md](build_and_layout.md), and [AGENTS.md](../AGENTS.md).

## 1. Outcome and implementation boundaries

Establish **`AlexandreAmice/robo_arch` as a private GitHub repository**, then deliver a first milestone demonstrating:

- Editable Python development through uv.
- C++23 components built by Bazel and immediately usable from Python simulations through nanobind.
- Bazel as the primary CI and testing interface.
- Independently composable scenario, autonomy, and world configurations.
- The same configured controller running in Drake and Isaac, including independent state and reset in an Isaac batch.

At planning time, the repository contains design documents and is not initialized with Git. GitHub authentication is available. Recheck those facts before R0 so setup is safe to resume.

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
- Component agents own their directories, including tests and local BUILD files. World agents own world integration and their component wrapper files.
- Coordinate edits to component declarations explicitly: wrapper agents provide registration changes to the component owner instead of concurrently rewriting the same file.
- An agent starts when its listed dependencies are merged. Earlier exploration is allowed, but it must not invent a competing interface.
- Every handoff states implemented behavior, public interface changes, commands actually exercised, limitations, and remaining dependencies.
- Merge after relevant checks pass. Require no independent reviewer approval that would prevent a solo maintainer from merging. Apply required checks through GitHub settings where supported.

Keep work-package status in this document; avoid creating a second architecture document for each agent. The coordinator updates status to prevent competing edits to this file.

## 3. Shared interfaces to establish before broad parallel implementation

### I0 — Minimal component and configuration interfaces

**Owner:** architecture/interface agent. **Dependencies:** R0.

Implement the smallest typed interfaces needed by the first tracking example:

| Interface | Required information |
|---|---|
| Scenario | References to robot, sensors, object instances, task, layout, and applicable installation calibration |
| Component declaration | Stable identifier, parameter schema, named input/output declarations, explicit world/variant factory registrations |
| Port description | Semantic quantity, robot/joints, units, frame, dimensions and timestamp convention |
| Implementation capabilities | Scalar/CPU-batch/tensor execution, device requirements, known transfers, timing and reset requirements |
| Resolved composition | Component instances, validated parameters, connections, exported ports and selected implementation references |
| World construction | Scenario realization, measurement/command bindings, and services supplied to selected factories |

Use Python types first; do not introduce a C++ type system merely to represent configuration.

The resolver produces inspectable data without starting a simulator or importing every SDK. World constructors consume this result through reusable assembly code. They must not contain per-example connection logic.

Leaf factories declare their ports. Composite interfaces derive recursively from explicitly exposed child ports. Support both nested compositions and a single monolithic policy declaration.

Timing and reset remain implementation responsibilities. For the first batch executor, support explicit sampled execution and reject unsupported immediate dependency cycles. Do not create a general scheduler or claim support for arbitrary Drake event semantics.

**Complete when:** small declarations express position and effort commands, nested composition, a monolithic policy interface, and explicit CPU support in Isaac without importing those runtimes.

## 4. Work packages and parallel execution

### Wave A — Independent foundation work

These packages can start together after R0.

**B0 — Build and dependency baseline**  
Own root build metadata and toolchain configuration. Establish Bzlmod, pinned C++23/Python toolchains, editable Python packaging, Ruff, Drake-derived clang-format settings, and Buildifier. Generate Bazel Python dependency inputs from the uv lock. Establish separate core, Drake, and Isaac environment profiles. Select exact dependency revisions through compatibility checks and commit the successful matrix; dependent packages consume those pins. Do not silently relax C++23 or the Python baseline. Coordinate the Isaac profile's runtime requirements with G0; G0 owns the vendor-environment files.

**I0 — Shared interfaces**  
Implement the interface package described above. It can develop alongside B0; its checks become Bazel targets once B0 lands.

**G0 — Local Isaac feasibility**  
Own the pinned vendor-environment description. Check the selected Isaac runtime's Python, driver, memory and GPU requirements, then attempt a minimal headless physics example without cameras. Record exact versions and startup/resource results. Use a separate environment from base development. Stop runtime expansion on a demonstrated compatibility/resource failure; report the blocker and continue non-GPU work. Do not change system drivers or the base development environment to force the probe to pass.

### Wave B — Infrastructure using the shared baseline

**B1 — Nanobind and native development bridge**  
**Depends on:** B0.  
Own binding build helpers, native wheel packaging, and the development helper.

Provide `native` and `run` operations, with an explicit environment profile:

- Ask Bazel to incrementally build the profile's native wheel.
- Check the active Python/native ABI combination.
- Install the resulting artifact without resolving dependencies again.
- Skip reinstalling an unchanged artifact only when the installed artifact matches.
- Launch the requested Python command in a fresh process.
- Stop on build/install failure rather than running stale native code.

Use a small numerical C++ library to prove the mechanism. Keep its Python binding thin and its C++ target independently linkable. The source-development profile must not pull in a competing released native wheel.

**Complete when:** changing C++ changes the result observed from editable Python, ordinary Python edits require no native rebuild, and a failed native build prevents launch.

**C0 — Bazel testing and GitHub Actions**  
**Depends on:** B0; add native packaging coverage after B1.  
Own CI workflows and shared pytest integration.

Run core tests through Bazel using declared sources, dependencies and resources. Add Drake checks after its integration exists. Reuse the same pytest sources under uv and Bazel. Include formatting checks and lock-export consistency checks.

Keep GPU jobs explicitly selected; a selected integration job must fail if its required runtime is unavailable. Do not count skipped GPU tests as Isaac validation. Keep remote caches and CI artifacts private.

**Complete when:** a fresh GitHub runner executes core Bazel checks without a developer `.venv`.

**S0 — Scenario and composition resolution**  
**Depends on:** B0, I0.  
Own configuration loading and resolution.

Implement safe YAML loading, relative references, parameter validation, recursive composition, port inference, implementation selection and diagnostics. Reject missing instances/frames, conflicting attachments, missing world implementations and incompatible commands. Report supported slow execution as a warning.

Include fixtures demonstrating:

- Robot/sensor replacement while retaining objects and task.
- A task referencing only a subset of scene objects.
- Layout replacement independently of object definitions.
- Installation-specific calibration.
- Nested and monolithic autonomy descriptions.

**Complete when:** both valid resolution and relevant failures are inspectable without loading Drake, ROS or Isaac.

**A0 — UR7e model and model loading**  
**Depends on:** I0; integrate build targets after B0.  
Own robot assets and model-loading code.

Pin the actual UR7e description with provenance. Establish named joints, frames, model parameters and actuator mappings. Keep control-model construction distinct from simulation state. Exclude the unspecified gripper from the first arm example rather than silently substituting one.

**Complete when:** world and controller construction consume the same identifiable robot description.

### Wave C — Shared computation and world integration

**K0 — Shared tracking components**  
**Depends on:** B0, I0, A0.  
Own the shared reference-generation and controller implementations.

First provide a small position-command tracking composition. Then add inverse dynamics using the existing Drake controller/dynamics implementation. Keep gains, reference conventions, force accounting and state/reset behavior shared.

World adapters must call this computation rather than reproducing controller equations. Keep Python where practical; use B1 when native batch invocation becomes necessary.

**Complete when:** controller behavior can be exercised independently of a simulator with explicit inputs and state.

**D0 — Drake world and generic assembly**  
**Depends on:** S0, A0, K0.  
Own Drake scene construction, generic Diagram assembly and Drake component wrappers.

Resolve the scenario into simulation models, measurement sources and accepted command ports. Build the declared autonomy recursively through registered factories. Keep the assembly reusable for later real-world driver substitution, following the separation illustrated by Drake's HardwareStation approach. [HardwareStation reference](https://manipulation.mit.edu/python/station.html)

Provide a Python-authored tracking run. Record configured rates, controller identity and effective parameters.

**Complete when:** the declared position stack runs, followed by inverse dynamics, without per-demo wiring inside the world implementation.

**X0 — Isaac world and generic assembly**  
**Depends on:** successful G0, S0, A0, K0.  
Own Isaac scene integration, execution and Isaac component wrappers.

Use the same resolved composition and shared components. Start with one arm and no camera. Explicitly map joint order and command modes. Disable or account for simulator drives that would add unintended control behavior.

Use explicitly registered scalar/CPU wrappers initially. Preserve implementation identities and report transfers. Do not replace inverse dynamics with an Isaac controller for convenience.

**Complete when:** the same configured controller operates in both worlds and produces corresponding outputs for matched inputs/state.

**D0 and X0 can run in parallel** once their shared dependencies land. Neither owns a separate controller implementation.

### Wave D — Batch reuse and milestone integration

**X1 — Batched shared execution**  
**Depends on:** B1, X0.

Extend Isaac execution from one environment to a small batch, starting with two. Give each environment independent controller state and support selective reset.

Use bulk CPU invocation of the same controller where needed. A native loop around existing Drake controller instances is acceptable; duplicating its control law is not. Keep tensor operations on-device where supported and report remaining transfers.

**Complete when:** resetting one environment leaves the others' controller state intact and the execution report accurately identifies CPU/scalar work.

**M0 — First milestone integration**  
**Depends on:** B1, C0, S0, D0, X0, X1.

Own the short user workflow and integrated examples. Exercise the complete C++ edit → rebuild/install → Python simulation loop, plus Bazel tests and installed-package checks. Record timings separately for compilation, installation, startup and controller execution; do not promise throughput without measurements.

**Complete when:** a fresh clone can reproduce the demonstrated workflows from documented commands.

### Suggested four-agent scheduling

| Stage | Agent 1 | Agent 2 | Agent 3 | Agent 4 |
|---|---|---|---|---|
| Bootstrap | Wait | Wait | Wait | Coordinator completes R0 |
| Foundation | B0 build | I0 interfaces | G0 Isaac feasibility | Coordination/review |
| Infrastructure | B1 native bridge | S0 resolver | A0 model | C0 CI |
| Execution | K0 shared components, then support | D0 Drake after K0 | X0 Isaac after K0 | Integration/review |
| Milestone | Native batch support | Drake correspondence checks | X1 batching | M0 integration |

With fewer agents, preserve dependencies and execute the same packages sequentially. With more agents, divide only packages with distinct file ownership. A scheduling row is not permission to start a task before its listed dependencies are ready.

### Following work packages

After controller reuse is established, extend the architecture through these independently owned tracks:

| Track | Deliverable | Dependencies |
|---|---|---|
| Task and scene | Nut/pin assets, layout, task participants and success evaluation; preserve the nut bore | S0 and actual part geometry |
| Hardware | UR ROS 2 interface, supported command mode, driver status and installation calibration | I0, A0 and selected hardware/software |
| Gripper and sensing | Selected Robotiq adapter, sensor sources, calibration and declared perception inputs | Exact device selections |
| Manipulation | Grasp, transport, insertion, release and recovery through the composed stack | Task/scene, gripper/sensing and appropriate control |
| Learning | Policy component, artifact metadata and training environment using the same autonomy implementation | Batched execution and selected observation/action requirements |
| Additional worlds | Newton and MuJoCo Warp implementations satisfying the same declarations | Existing resolver and execution interfaces |

Task/scene and hardware work can proceed independently. Hardware-specific implementation waits for the actual gripper, sensors, dimensions and driver capabilities; agents must not invent those specifications. Learner choice and GPU optimization receive their own plans once the workload exists.

## 5. Acceptance gates

**Repository gate:** private upstream exists, initial commit is pushed, and fresh-clone instructions are correct.

**Development gate:** editable Python works without Bazel; native changes reach Python through one command; ABI mismatch and failed builds stop clearly; installed artifacts work outside the checkout.

**Configuration gate:** robot, sensors, task objects and layout can vary independently; nested and monolithic stacks resolve; incompatible ports/world support fail with instance-specific diagnostics.

**Reuse gate:** Drake and Isaac select the same shared controller/model/parameters. Compare outputs using identical inputs and state with documented numerical tolerances. Closed-loop trajectories may differ between engines.

**Batch gate:** at least two environments run with independent state and selective reset. CPU/GPU transfer and scalar-execution warnings are visible. Throughput is measured, not inferred from annotations.

**CI gate:** Bazel runs the authoritative CPU/Drake checks without consuming `.venv`; selected GPU checks execute in their recorded environment. Wheel installation and resource loading receive separate coverage.

If local Isaac cannot run, publish the completed repository, development workflow and Drake results with an explicit blocked Isaac gate. Do not mark the controller-reuse milestone complete until actual Isaac execution evidence exists.

## Work-package status

Status as of 2026-09-28. This batch covers R0, B0, I0 and G0 only.

| Package | Status |
|---|---|
| R0 | Complete: private upstream, matching commits, fresh clone and Actions availability verified |
| B0, I0, G0 | In progress in separate branches, worktrees and environments |
| B1, C0, S0, A0 | Await listed dependencies |
| K0, D0, X0 | Await listed dependencies |
| X1, M0 | Await listed dependencies |
