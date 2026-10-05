# Robotics architecture

Local robotics development with shared autonomy across Drake and Isaac Lab (PhysX or Newton/MuJoCo Warp). Examples
exercise a UR7e with a RealSense D435 housing, an iiwa 7 with an ATI Mini45-R, and
a mixed UR7e–iiwa bimanual assembly with two independent force/torque sensors.
Both robots and both sensors have physical collision geometry.

This project is open source under the [BSD 3-Clause License](LICENSE), the same
permissive license used by Drake. Third-party components retain their own
licenses as documented beside their source or assets.

The [documentation index](docs/README.md) connects the architecture, build guide,
API reference, examples and model provenance. For commands to render the shared
Python/C++ reference, see [API documentation](docs/build_and_layout.md#api-documentation).

## Local development

Use the pinned uv version in `pyproject.toml` and Bazelisk (`.bazelversion`):

```sh
uv sync --locked
uv run tools/native/install.py --profile drake
uv run --group docs pytest
bazel test //src/robo_arch/... //tests/build:core //tests/build:cxx23 //tests/build:drake
uv run ruff check .
uv run ruff format --check .
uv lock --check
bazel run //:buildifier
```

There are no hosted CI workflows or required remote checks. Python edits remain
editable through uv; Bazel builds C++ and offers an independent local test path.
Tests use importlib collection so owner-local tests can share filenames.
Select `.venv/bin/python` in your IDE. Core declarations can be inspected in an
SDK-independent environment using `uv sync --locked --no-group drake`.

Direct scenario scripts incrementally build `//tools/native:wheel` and refresh it in the
selected environment before running. Unchanged payloads are not reinstalled;
build/install failures stop the launch. Exact `uv sync` can remove the wheel;
the next scenario launch restores it automatically. The explicit installer in
the test setup above is also available for IDEs and notebooks. See the
[controller](src/robo_arch/core/controllers/joint_pd/README.md) for ownership,
units and ABI details.

Component libraries and bindings stay with their owners; the project package
collects their explicitly declared extensions and runtime resources. Adding a
native component does not change launch/install code. See the
[native edit–run workflow](docs/build_and_layout.md#the-c-editrun-loop).

For C++ autocomplete and navigation in VS Code, install the recommended
**clangd** extension (`llvm-vs-code-extensions.vscode-clangd`). Open the
repository/worktree root and run:

```sh
bazel run //:refresh_compile_commands
```

This uses [Hedron's extractor](https://github.com/hedronvision/bazel-compile-commands-extractor)
to generate an ignored `compile_commands.json` with Bazel's C++23 flags and
toolchain paths, plus an ignored `external` link into Bazel's dependencies. The
workspace settings use clangd bundled with the pinned LLVM toolchain and disable
Microsoft C++ IntelliSense to avoid duplicate diagnostics. After the first run,
use **clangd: Restart language server** if clangd started before the toolchain
was available. Regenerate after changing BUILD files, dependencies or compiler
options, and separately in each
worktree. Pass extra build flags after `--`, for example
`bazel run //:refresh_compile_commands -- --compilation_mode=dbg`.

## Run the examples

Use `uv run src/robo_arch/scenarios/<scenario>/run.py` for either simulator. The script selects
the environment from the run configuration (including `--world`, `--world-config`
or saved `--inspect` inputs) and refreshes native code before launching. Append
`--help` for scenario options; help does not build or import simulators.
`--run` selects a YAML file or package URI; named flags such as `--duration 5`
override its settings. `--world isaac` replaces the world with Isaac defaults,
while `--world-config` selects a complete world YAML. Environments must be synced
once as shown here. Switching worlds does not add unsupported scenario behavior.

```sh
# UR7e, detailed meshes, physical D435 and ideal RGB-D rendering in Drake.
uv run src/robo_arch/scenarios/arm_tracking/run.py

# Seven-axis native PD + gravity feedforward, with wrist force/torque sensing.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/iiwa7.yaml

# Different arm models and independent controllers/sensors in one nested system.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml

# Deliberate sensor-face contact with a fixed block; evaluates measured force.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/iiwa7_contact.yaml
```

Drake saves and opens interactive Meshcat playback. Use `--record <path.html>`
to choose the destination, `--no-browser` to suppress opening it, or `--headless`
for local automated checks. Live proximity/contact inspection uses
`--visualization live_and_record`; select the layers in Meshcat's controls.

Each CLI run saves resolved configuration, source/asset hashes, versions and an
inspection command. NPZ traces and PNG plots contain measured positions, efforts
and wrenches. `--inspect <report.json>` restores inputs using the current code
and assets; it does not overwrite the original recording.

Isaac uses a separate pinned environment:

```sh
uv sync --project third_party/isaac --locked

# Sixteen independent UR7e reaching environments, with native viewing.
uv run src/robo_arch/scenarios/batched_reaching/run.py --backend newton --live --hold

# The same scalar tracking command, selecting Isaac instead of Drake.
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world isaac --headless --metadata recordings/bimanual_isaac.json
```

For a native desktop view, keep `DISPLAY` set and use:

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/desktop.yaml
```

This selects CPU PhysX and Kit's Storm renderer, retains the final scene for
inspection, and saves a viewport PNG. Close the window or press Ctrl-C to exit.
The script sets `OMNI_KIT_ACCEPT_EULA=YES` (accepting NVIDIA's runtime EULA)
unless already set, and removes display variables for headless Isaac runs.
Both new systems and the contact example support Isaac force/torque sensing. D435 image generation remains
Drake-only: the original camera-equipped example needs `--no-sensors` in Isaac,
which retains its physical housing and inertia. See [Isaac setup and limits](third_party/isaac/README.md).

These are nominal simulation examples. Robot collision uses model-specific mesh
hulls; the Mini45's segmented geometry preserves its bore. Ideal sensing,
estimated sensor inertias and nominal mounts are documented beside each device.
The [batched reaching example](src/robo_arch/scenarios/batched_reaching/README.md)
runs tensor PD, independent goals and selective resets on configurable PhysX or
Newton/MuJoCo Warp backends, with a local throughput comparison and native viewing.
The [camera protection example](src/robo_arch/scenarios/camera_protection/README.md)
runs the shared sphere CBF in Drake or with optional Torch/Moreau CUDA control
on Isaac Lab PhysX/PGS. Its guide includes the optional solver setup and commands.
There is no hardware execution, gripper, nut placement or RTX viewer. Scalar CPU
controllers are reported explicitly; shared code does not imply identical
simulator contact forces.
