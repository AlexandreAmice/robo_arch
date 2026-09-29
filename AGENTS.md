# Project guidance

## Scope and communication

- Follow the user's current role and task. Design work means documentation, not implementation, tests or builds.
- These documents are proposals. Preserve user decisions and edits; distinguish requirements from suggestions. Shared chats and neighboring repositories are background, not approved designs.
- Use established terminology. Do not invent names such as `RobotProgram` for the autonomy stack or add abstractions without a concrete responsibility.
- Keep documentation short. Update the existing authoritative section rather than adding overlapping proposals. Remove stale statements when decisions change. Keep exact APIs and dependency pins provisional until chosen.
- Make routine authorized changes without repeated permission requests. Report outcomes, material limitations and what was actually checked concisely.

## Design constraints

- Follow [architecture.md](docs/architecture.md) and [build_and_layout.md](docs/build_and_layout.md). The first application is UR7e + Robotiq, placing nuts onto a pin; hardware details remain open.
- Reuse autonomy code across deployment and training. Use explicit world implementations, preferably thin wrappers over shared code. Missing support is an error; supported scalar/CPU execution in batched simulation produces warnings. Intentional approximations must be named.
- YAML configuration references use `package://robo_arch/...` resources, never paths relative to another YAML file or the working directory.
- Timing and reset belong primarily to implementations. Avoid a new universal scheduler, YAML programming language or speculative performance framework.
- Follow the file-placement rules below. Actuated tools are robots; physical assembly and autonomy composition remain distinct.
- Keep reusable algorithms separate from robot/system/scenario specializations. Physical assembly does not fix an autonomy stack. Calibration belongs with the device or relationship it describes, with explicit instance identity and profile selection. Do not require optional SDKs to inspect declarations.

## File placement

Application source belongs under `src/robo_arch/`. Reusable declarations and configuration records live in `core/contracts/` and `core/config/`. Add other packages at the locations below as their implementations are introduced.

| Location | Contents |
|---|---|
| `robots/<robot>/` | Robot/tool-specific controllers, IK, drivers, model assets, calibration profiles and tests |
| `sensors/<sensor>/` | Sensor-specific code, assets, per-unit calibration and tests |
| `robot_system/<system>/` | Device/child-system composition, mounts, assembly calibration, coordinated autonomy and tests |
| `objects/<object>/` | Object geometry, physical metadata and provenance |
| `scenarios/<scenario>/` | System/object selection, task and evaluation, scene layout, fixture calibration, task-specific autonomy, run configs and tests |
| `core/<responsibility>/` | Reusable algorithms, contracts, config loading, model utilities, world assembly, learning and visualization; no concrete device/scenario imports |

- Keep C++, Python bindings, YAML and assets beside their owner. Put tests in that owner's `tests/`; root `tests/` is for cross-package integration and installed artifacts.
- Device-specific `drake/`, `isaac/` and `real/` code stays with the device. Generic world/transport services belong under `core/worlds/`. Do not duplicate algorithms between these locations.
- Keep robot-independent inverse dynamics in `core/controllers/`; keep robot model selection, gains and specializations with the robot, system or scenario that owns those assumptions.
- Root `tools/` contains developer/build utilities and diagnostics;
  `third_party/` contains dependency metadata, independently locked vendor
  profiles and patches; `deployment/` contains deployable runtime images and
  launch material. None is a second home for device implementations.
- Put maintained design/user documentation in the existing `docs/` pages or an owning package's README. Create directories only when they have content; avoid catch-all `utils/` and duplicate asset/configuration trees.

## Cleanup and maintained documentation

- Leave a maintainer-facing result, not an agent work diary. Do not put agent names, work-package labels, completion claims, handoffs or chronological command transcripts in source, README files or dependency notes. Work-package identifiers and status belong only in `docs/implementation_tasks.md` or PR descriptions.
- Keep durable information: how to use the code, dependencies, compatibility limits, reproducible diagnostics and measured results that affect a decision. Explain them without requiring knowledge of the implementation assignment. Do not repeat pass/fail checklists across documents.
- Use ignored `plans/` or temporary storage for scratch notes and raw output. Before handing off, remove temporary files created for the task, abandoned approaches, stale comments, broken references and debug scaffolding. Keep a diagnostic script only when it has an ongoing purpose and a documented owner/location.
- Inspect the final diff and report only changes that actually remain. Do not delete user work, other agents' active files, useful environments or caches merely to make the tree look clean. No tests or builds are needed for prose/comment-only cleanup.

## Languages and style

Prefer Python for experimentation and most autonomy work. Add C++ when requested, performance needs are demonstrated, or stable code benefits enough to justify its maintenance cost.

- **C++23**, following [Drake's modified Google C++ style](https://drake.mit.edu/styleguide/cppguide.html). Our language-version requirement takes precedence over the guide's version restriction. Use `.h`/`.cc`, self-contained headers with `#pragma once`, two-space indentation and an 80-column target. Pin clang-format using Drake's configuration adapted to project paths.
- Prefer explicit ownership and straightforward interfaces. Constrain templates with small concepts or `requires` clauses when useful; prefer these over SFINAE or constraint-like `static_assert`s. Scope third-party compatibility settings to the affected dependency.
- **Python 3.12 minimum** for shared source; pin interpreters per environment. Any older vendor interpreter needs an explicit compatibility decision.
- Use **Ruff for linting and formatting**, configured in `pyproject.toml`: `py312`, line length 88, four-space indentation, double quotes; initial rules `E4`, `E7`, `E9`, `F`, `I`, `UP`, `B`. Do not add overlapping formatters/linters.
- Use modern annotations on public and nontrivial interfaces, standard Python naming, and explicit units, frames, timestamps and array ownership. Keep suppressions narrow.
- Raise exceptions for errors and let them propagate; catch them only in tests. Use Python `finally` blocks or C++ RAII for required cleanup.

## Build and validation

- Use Bazel/Bzlmod with explicit dependencies, pinned toolchains, narrow visibility and Buildifier. Learn from Drake and `../gcs_solver_project` without copying whole build frameworks or local paths.
- Keep nanobind bindings thin and C++ libraries independently usable. Document ownership, layout, GIL behavior and ABI constraints. Use uv for ordinary Python development, Bazel for native compilation and primary CI/testing, and the same source/tests in both workflows. Keep the native rebuild-to-Python workflow explicit and easy.
- Add only meaningful implementation tests; keep checks proportional to the change. Documentation needs no tests, builds or simulator launches. Format changed first-party code without unrelated reformatting.
- Attach a visualization of the actual run whenever asking the user to test or inspect it, with a command to launch it locally. Use headless validation primarily for automated tests. Make failing tests easy to inspect visually using the same inputs and state; follow the [testability requirements](docs/build_and_layout.md#testability).
- Distinguish measured from expected performance and simulation from hardware evidence. Do not claim that shared code guarantees GPU efficiency or identical physics.
