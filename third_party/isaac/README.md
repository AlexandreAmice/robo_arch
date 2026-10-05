# Isaac Lab dependency profile

This independently locked Python 3.12 environment uses Isaac Lab 3.0 Early Access
(`isaaclab==3.0.0rc1`) with Isaac Sim 6.1/PhysX and Newton 1.5.2/MuJoCo Warp. It runs fixed-base UR7e and iiwa 7
articulations, including mixed bimanual composition and ATI Mini45 force/torque
sensors. Both worlds use the same packaged physical models and controller code.
The vendor environment does not enter the root dependency resolution.

## Local execution

```sh
uv sync --project third_party/isaac --locked
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world isaac --headless --metadata recordings/bimanual_isaac.json
```

The scenario script selects the vendor interpreter, refreshes native code and sets
`OMNI_KIT_ACCEPT_EULA=YES` (accepting NVIDIA's runtime EULA) unless already set.
It removes display variables only for headless Isaac runs. The same command accepts
`iiwa7.yaml` and `iiwa7_contact.yaml`. The original D435 example requires
`--no-sensors`: its housing is supported, but image generation is not. Disabling
observations retains physical devices and their inertia/collisions.

A complete `--world-config` can select CPU/PGS instead of default GPU/TGS:

```yaml
type: isaac
physics:
  time_step: 0.001
  solver: pgs
  device: cpu
visualization:
  type: isaac
  mode: "off"
```

For Newton, select `package://robo_arch/core/worlds/isaac/newton.yaml` or set
`physics.backend: newton` and `physics.solver: mujoco_warp`. Newton is initially
supported on CUDA. Newton sensor observations are rejected before startup;
its pinned joint-wrench implementation excludes fixed sensing joints. Use
`--no-sensors` for scalar tracking with Newton to retain mounted physical bodies.
Backend-specific settings are validated before SDK imports.
The [batched reaching example](../../src/robo_arch/scenarios/batched_reaching/README.md)
uses tensor PD, explicit simulator-gravity feedforward, independent goals and
masked reset on both engines. It includes live inspection and reproducible
1/16/64-environment throughput comparisons. Existing dependency pins suffice;
Newton/MuJoCo Warp were already included by the Lab wheel.

## Native viewing

Run from a desktop terminal, keeping `DISPLAY` set:

```sh
uv run src/robo_arch/scenarios/arm_tracking/run.py \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/desktop.yaml \
  --metadata recordings/bimanual_isaac_live.json
```

The minimal Kit profiles include Lab’s OmniPhysics stage-update and PhysX
backend extensions, which CPU warmup requires. The desktop profile selects
CPU/PGS physics and a native Kit **Storm** viewport.
The window displays the actual simulated stage, paced toward real time, and stays
open at the final state for camera navigation. Close the window or press Ctrl-C
in the terminal. Each live CLI run saves `<report>.viewport.png` before holding
for inspection. `--inspect <report.json> --visualization live` reruns the same
resolved inputs with this viewer. Select GPU/TGS physics with
`--world isaac --visualization live` instead of `--world-config`. This is live
inspection, not a video recording.

Storm uses NVIDIA OpenGL. The runner selects NVIDIA PRIME/GLX defaults for
hybrid desktops before starting Kit, while honoring existing environment values.
It requires X11 or XWayland; there is no browser or remote streaming viewer.
Rendering and presentation run synchronously; the independent Kit 110.3 present
thread is disabled because it raced renderer teardown. USD transforms publish
at the display cadence, and Kit updates do not advance physics implicitly.
Drawing pauses and GPU work drains before Lab closes the stage; a second wait
follows Lab’s final Kit updates before releasing the viewport handles.
Collision guides are hidden from illustration without removing their physics.
Closing the window defers shutdown until traces are saved and physics detaches.

RTX viewing and camera rendering remain unvalidated on this machine. The earlier
direct-Sim RTX attempt did not finish its first frame within roughly ten minutes
of shader compilation. Storm does not enable D435 images, RTX sensors, native
collision/contact overlays or viewport video recording.

The Mini45's explicit convex collision sectors need more GPU broadphase aggregate
pairs than the vendor default. `gpu_found_lost_aggregate_pairs_capacity` defaults
to 32768; this is a native physics setting, not a batching-capacity promise.

The runner saves configuration/results/errors, an inspection command, and
measured NPZ state/effort/wrench traces with PNG plots. Inspection uses current
code/assets and writes separate artifacts. There is no automatic replacement
with Drake playback.

## Compatibility and limits

Local Isaac Lab validation on October 4, 2026 exercised UR7e inverse dynamics,
iiwa PD, mixed-arm control and deliberate contact on GPU/TGS, plus mixed-arm
tracking on CPU/PGS. The tracking
examples reached their existing tolerances (maximum final joint errors:
UR7e `0.000167` rad, iiwa `0.000113` rad, mixed arms `0.000241` rad). The contact
example measured a `389.2` N peak ideal reaction force. A two-environment mixed
system passed selective reset checks for joint state, commanded effort, sensor
samples, episode clocks and controller contexts; root poses and collision groups
were also checked. These are simulation results, not hardware or throughput
evidence. Reproduction and visual diagnosis use the native integration tests in
the [scenario README](../../src/robo_arch/scenarios/arm_tracking/README.md).

Newton also passed mixed-arm scalar tracking with observations explicitly
disabled (maximum final joint error 0.000127 rad). Physical sensor bodies were
retained; this does not validate Newton wrench observations.

The RTX 3060 Laptop has 6 GB VRAM and still fails the vendor VRAM requirement.
The camera-free reaching example has also exercised 64 environments on each
backend; its owning README records throughput and the measurement limits.
This does not establish camera capacity or capacity beyond the tested workloads. Native Storm viewing is available; RTX rendering and cameras remain
separate compatibility work. Native Storm runs on CPU/PGS, GPU/TGS and the
two-environment reset case each matched all 3,001 headless state/effort/wrench
samples exactly, with viewport captures and normal process exit.

`batch.yaml` selects two collision-isolated copies of the complete assembly.
Lab owns articulation/sensor buffers and simulation stepping; selective reset
also reconstructs controller contexts and restarts each selected episode clock.
USD cloning with full PhysX parsing is intentional for these small batches.

Arm-tracking control is scalar CPU: joint state and effort cross the GPU boundary each step
when GPU physics is selected. URDF-to-USD conversion runs in a child process to
isolate incompatible USD libraries. Mounted sensor bodies and their sensing
joints are included before conversion; device adapters map observations by link
identity. Temporary converted assets are removed after execution.

Isaac Sim is pinned to 6.1.0.0 and PhysX/tensors to 110.3.2. The Lab wheel
comes from NVIDIA’s package index. Its published dependency overrides are
explicit in `pyproject.toml`; shared dependencies remain outside the root lock.
See the [3.0 EA installation instructions](https://isaac-sim.github.io/IsaacLab/v3.0.0-EA/source/setup/installation/index.html). `uv.lock` pins Python
packages, while Kit downloads additional extensions into its cache; this profile
is not offline/hermetic. The native controller uses hidden static-library symbols
to avoid interposition with Kit libraries. Do not replace those linkage settings
with globally exported C++ runtime symbols.

For Lab 3.0.0rc1, teardown stops the timeline and releases retained manager and
launcher handles before Kit unloads plugins. Kit’s path-stat cache is disabled:
its cached import exceptions retain scene frames and native views past teardown.
Native stdout/stderr interception is also disabled: libraries can retain those
stream handles beyond Kit shutdown, retaining unloaded message queues.
On failure, completed traceback-frame locals are cleared; exception messages and
stack locations propagate, while partial NPZ traces retain simulation state.

The independent GPU free-fall diagnostic remains under `tools/isaac/`:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python tools/isaac/probe.py
```

## Optional GPU camera controller

Install the Moreau CUDA solver into this same locked Lab environment:

```sh
uv sync --project third_party/isaac --locked --group cbf-gpu --group test
uv run src/robo_arch/scenarios/camera_protection/run.py \
  --world-config package://robo_arch/scenarios/camera_protection/isaac_gpu.yaml \
  --backend torch_moreau --batch-size 2 --compile-model --no-browser
```

The optional group adds Moreau 0.4.1 with CUDA 13 support; Lab's Torch 2.11 and
backend compatibility overrides remain authoritative. Camera protection uses
Lab's PhysX/PGS runtime, not a separate native PhysX loop. It retains mounted
camera bodies with observations disabled. See the
[scenario guide](../../src/robo_arch/scenarios/camera_protection/README.md) for
numerical limitations, recorded evidence and native integration checks.
