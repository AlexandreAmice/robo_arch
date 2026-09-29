# Robotics architecture

A shared autonomy stack across Drake, hardware and batched simulation. The first
runnable example tracks a joint target with a UR7e in Drake and renders a box
through an idealized wrist RGB-D camera. Cross-world controller reuse remains
future work.

- [Architecture](docs/architecture.md): scenario, autonomy, world, composition and shared execution.
- [Build and layout](docs/build_and_layout.md): device-owned code/assets, recursive robot systems, scenarios, core libraries, and Python/C++ packaging.
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
uv lock --check
```

The default `.venv` includes development tools and Drake. For an SDK-independent
environment, use `uv sync --locked --no-group drake` and the same
`--no-group drake` option with `uv run`.

In VS Code, install Microsoft's Python and Python Debugger extensions and select
`.venv/bin/python` with **Python: Select Interpreter**. The workspace setting
provides this default for new selections; change any previously selected
interpreter explicitly. Open `src/robo_arch/scenarios/arm_tracking/run.py` and
use **Run Python File** or **Python Debugger: Debug Python File** from its run
button dropdown. Python edits require no Bazel build.

Bazel uses its own pinned Python dependencies and C++23 toolchain:

```sh
bazel test //tests/build:core //tests/build:cxx23 \
  //src/robo_arch/core/worlds:registry_test \
  //src/robo_arch/core/config:loading_test
bazel run //:buildifier
```

Run the example and open its interactive scene playback:

```sh
uv run --locked python -m robo_arch.scenarios.arm_tracking.run
bazel run //src/robo_arch/scenarios/arm_tracking:run
```

It saves `recordings/arm_tracking_drake.html`, opens it in your browser, and prints joint
positions, tracking error and camera depth-pixel counts. Camera images are saved
beside the playback. Use `--no-browser` to save
playback without opening it, or `--headless` for automated checks. Select another
run with `--run path/to/run.yaml` or a `package://robo_arch/...` URI. The packaged
[scenario](src/robo_arch/scenarios/arm_tracking/scenario.yaml) keeps the world,
robot-system and controller selection, object poses, task target and gains together.
Only the reusable physical assembly lives in a separate YAML file. The
[configuration guide](src/robo_arch/scenarios/arm_tracking/README.md) explains
the fields, units, defaults and file references.

The example exercises each owner:

- `robots/ur7e/`: nominal model and Drake adapter.
- `sensors/ideal_camera/`: noiseless pinhole RGB-D rendering.
- `robot_system/ur7e_ideal_camera/`: arm and wrist mount.
- `objects/box/`: self-contained object asset.
- `scenarios/arm_tracking/`: layout, task, tuning, run and evaluation.
- `core/`: YAML loading, inverse-dynamics control and physical scene assembly.

Run the same implementation tests through either workflow:

```sh
uv run --locked pytest
bazel test //src/robo_arch/... //tests/build:core //tests/build:drake
```

See the [dependency baseline](third_party/compatibility.md) for pins and ABI
boundaries, and [Isaac feasibility](deployment/isaac/README.md) for the isolated
vendor environment and measured results. Native wheel installation and the
C++ edit–run helper are not implemented yet.

The loader supports nested physical systems, strict YAML fields and package
references without importing Drake. Execution currently supports one fixed-base
arm with inverse-dynamics tracking and fixed scene objects. The controller uses a
separate robot dynamics model and ideal joint measurements; the camera is
observed but does not guide control. The UR7e has simplified visuals and no robot
collision geometry. This example does not perform grasping or nut placement.

Controller implementations wire their robot observations and commands once;
scenario Python supplies task references through the exposed native ports. YAML
supplies selections and parameters, not an execution graph. Device definitions
are discovered from the selected packages rather than listed in each scenario. Measured
calibration, batched execution and performance warnings remain planned. Unsupported
worlds fail explicitly; the Drake example does not establish hardware or Isaac
support. Camera rendering requires an OpenGL context; headless EGL works on the
development host.
