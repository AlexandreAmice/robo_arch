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
- Timing and reset belong primarily to implementations. Avoid a new universal scheduler, YAML programming language or speculative performance framework.
- Keep reusable algorithms separate from application tuning. Co-locate related application configurations. Do not require optional SDKs to inspect declarations.

## Languages and style

Prefer Python for experimentation and most autonomy work. Add C++ when requested, performance needs are demonstrated, or stable code benefits enough to justify its maintenance cost.

- **C++23**, following [Drake's modified Google C++ style](https://drake.mit.edu/styleguide/cppguide.html). Our language-version requirement takes precedence over the guide's version restriction. Use `.h`/`.cc`, self-contained headers with `#pragma once`, two-space indentation and an 80-column target. Pin clang-format using Drake's configuration adapted to project paths.
- Prefer explicit ownership and straightforward interfaces. Constrain templates with small concepts or `requires` clauses when useful; prefer these over SFINAE or constraint-like `static_assert`s. Scope third-party compatibility settings to the affected dependency.
- **Python 3.12 minimum** for shared source; pin interpreters per environment. Any older vendor interpreter needs an explicit compatibility decision.
- Use **Ruff for linting and formatting**, configured in `pyproject.toml`: `py312`, line length 88, four-space indentation, double quotes; initial rules `E4`, `E7`, `E9`, `F`, `I`, `UP`, `B`. Do not add overlapping formatters/linters.
- Use modern annotations on public and nontrivial interfaces, standard Python naming, and explicit units, frames, timestamps and array ownership. Keep suppressions narrow.

## Build and validation

- Use Bazel/Bzlmod with explicit dependencies, pinned toolchains, narrow visibility and Buildifier. Learn from Drake and `../gcs_solver_project` without copying whole build frameworks or local paths.
- Keep nanobind bindings thin and C++ libraries independently usable. Document ownership, layout, GIL behavior and ABI constraints. Use uv for ordinary Python development, Bazel for native compilation and primary CI/testing, and the same source/tests in both workflows. Keep the native rebuild-to-Python workflow explicit and easy.
- Add only meaningful implementation tests; keep checks proportional to the change. Documentation needs no tests, builds or simulator launches. Format changed first-party code without unrelated reformatting.
- Distinguish measured from expected performance and simulation from hardware evidence. Do not claim that shared code guarantees GPU efficiency or identical physics.
