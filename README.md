# Robotics architecture

Local robotics development with shared autonomy across Drake and Isaac. Examples
exercise a UR7e with a RealSense D435 housing, an iiwa 7 with an ATI Mini45-R, and
a mixed UR7e–iiwa bimanual assembly with two independent force/torque sensors.
Both robots and both sensors have physical collision geometry.

- [Architecture](docs/architecture.md): composition, autonomy and explicit worlds.
- [Build and layout](docs/build_and_layout.md): ownership and local Python/C++ workflow.
- [API documentation](docs/build_and_layout.md#api-documentation): build the combined Python and C++ reference locally.
- [Implementation status](docs/implementation_tasks.md): implemented capabilities and remaining work.
- [Example configuration](src/robo_arch/scenarios/arm_tracking/README.md): runs, units and inspection.

## Local development

Use the pinned uv version in `pyproject.toml` and Bazelisk (`.bazelversion`):

```sh
uv sync --locked
uv run tools/dev.py native --profile drake
uv run pytest
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

The native helper builds and installs a private wheel into the selected existing
environment. It skips unchanged installed bytes, stops on failure and launches a
fresh process for `run`. Exact `uv sync` can remove the development wheel; rerun
the helper afterward. See the [controller](src/robo_arch/core/controllers/joint_pd/README.md)
for ownership, units and ABI details.

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

```sh
# UR7e, detailed meshes, physical D435 and ideal RGB-D rendering in Drake.
uv run python -m robo_arch.scenarios.arm_tracking.run

# Seven-axis native PD + gravity feedforward, with wrist force/torque sensing.
uv run tools/dev.py run --profile drake -- python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/iiwa7.yaml

# Different arm models and independent controllers/sensors in one nested system.
uv run tools/dev.py run --profile drake -- python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml

# Deliberate sensor-face contact with a fixed block; evaluates measured force.
uv run python -m robo_arch.scenarios.arm_tracking.run \
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
uv run tools/dev.py native --profile isaac
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world isaac --headless --metadata recordings/bimanual_isaac.json
```

For a native desktop view, keep `DISPLAY` set and use:

```sh
OMNI_KIT_ACCEPT_EULA=YES third_party/isaac/.venv/bin/python \
  -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/desktop.yaml
```

This selects CPU PhysX and Kit's Storm renderer, retains the final scene for
inspection, and saves a viewport PNG. Close the window or press Ctrl-C to exit.
The environment variable accepts NVIDIA's runtime EULA. Both new systems and the
contact example support Isaac force/torque sensing. D435 image generation remains
Drake-only: the original camera-equipped example needs `--no-sensors` in Isaac,
which retains its physical housing and inertia. See [Isaac setup and limits](third_party/isaac/README.md).

These are nominal simulation examples. Robot collision uses model-specific mesh
hulls; the Mini45's segmented geometry preserves its bore. Ideal sensing,
estimated sensor inertias and nominal mounts are documented beside each device.
There is no hardware execution, gripper, nut placement, batched rollout or RTX
viewer. Scalar CPU controllers are reported explicitly; shared code does
not imply GPU-efficient control or identical simulator contact forces.
