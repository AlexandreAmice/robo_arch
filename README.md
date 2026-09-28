# Robotics architecture

A shared autonomy stack across Drake, hardware and batched simulation, starting
with build tooling and SDK-independent declarations. Controller execution and
the controller-reuse milestone remain future work.

- [Architecture](docs/architecture.md): scenario, autonomy, world, composition and shared execution.
- [Build and layout](docs/build_and_layout.md): application-grouped configuration, component folders, Python/C++ and packaging.
- [Implementation plan](docs/implementation_tasks.md): private GitHub setup, parallel work packages, dependencies, and controller-reuse acceptance gates.
- [Agent guidance](AGENTS.md): scope, concise documentation and coding style.

Clone the private repository with an authorized GitHub account:

```sh
gh repo clone AlexandreAmice/robo_arch
cd robo_arch
```

Use the pinned uv version in `pyproject.toml` and Bazelisk (which reads
`.bazelversion`). Python development needs no Bazel invocation:

```sh
uv sync --locked
uv run --locked pytest
uv run ruff check .
uv run ruff format --check .
uv run python tools/export_requirements.py --check
```

Bazel uses its own pinned Python dependencies and C++23 toolchain:

```sh
bazel test //tests/build:core //tests/build:cxx23 \
  //src/robo_arch/contracts:contracts_test \
  //src/robo_arch/config:descriptions_test
bazel run //:buildifier
```

The optional Drake dependency check uses a separate environment:

```sh
UV_PROJECT_ENVIRONMENT=.venv-drake uv sync --locked --group drake
UV_PROJECT_ENVIRONMENT=.venv-drake uv run --locked --group drake \
  pytest tests/build/smoke_drake.py
bazel test //tests/build:drake
```

See the [dependency baseline](third_party/compatibility.md) for pins and ABI
boundaries, and [Isaac feasibility](deployment/isaac/README.md) for the isolated
vendor environment and measured results. Native wheel installation and the
C++ edit–run helper belong to B1 and are not implemented yet.

Declarations live in `src/robo_arch/contracts/`; configuration descriptions,
resolved records and factory context live in `src/robo_arch/config/`. Their tests
show nested composition, input fan-out and a monolithic policy without importing
simulator SDKs. Records describe interfaces; S0 will implement loading,
resolution, compatibility errors and performance warnings.
