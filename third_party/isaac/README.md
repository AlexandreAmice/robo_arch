# Isaac dependency profile

This independently locked Python 3.12 environment runs fixed-base UR7e and iiwa 7
articulations, including mixed bimanual composition and ATI Mini45 force/torque
sensors. Both worlds use the same packaged physical models and controller code.
The vendor environment does not enter the root dependency resolution.

## Local execution

```sh
uv sync --project third_party/isaac --locked
uv run tools/dev.py native --profile isaac
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world isaac --headless --metadata recordings/bimanual_isaac.json
```

`OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's runtime EULA. The same command accepts
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

## Native viewing

Run from a desktop terminal, keeping `DISPLAY` set:

```sh
OMNI_KIT_ACCEPT_EULA=YES third_party/isaac/.venv/bin/python \
  -m robo_arch.scenarios.arm_tracking.run \
  --run package://robo_arch/scenarios/arm_tracking/bimanual.yaml \
  --world-config package://robo_arch/core/worlds/isaac/desktop.yaml \
  --metadata recordings/bimanual_isaac_live.json
```

The desktop profile selects CPU/PGS physics and a native Kit **Storm** viewport.
The window displays the actual PhysX stage, paced toward real time, and stays
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
at the display cadence, and the timeline never advances physics implicitly. Image writing and
GPU rendering finish before the stage and renderer are released. Collision guides
are hidden from normal illustration without removing their physics. Closing the
window defers Kit shutdown until traces are saved and the physics stage detaches.

RTX viewing remains unvalidated: the local RTX retry did not finish its first
frame within roughly ten minutes of shader compilation. No driver or system
configuration changes were made. Storm viewing does not enable D435 images,
RTX sensors, native collision/contact overlays or viewport video recording.

The Mini45's explicit convex collision sectors need more GPU broadphase aggregate
pairs than the vendor default. `gpu_found_lost_aggregate_pairs_capacity` defaults
to 32768; this is a native physics setting, not a batching-capacity promise.

The runner saves configuration/results/errors, an inspection command, and
measured NPZ state/effort/wrench traces with PNG plots. Inspection uses current
code/assets and writes separate artifacts. There is no automatic replacement
with Drake playback.

## Compatibility and limits

Local validation on September 30, 2026 exercised iiwa, mixed-arm and deliberate
contact runs with GPU/TGS, and mixed-arm tracking with CPU/PGS. Both arms tracked
their distinct targets, both wrist sensors produced independent measurements,
and the processes shut down cleanly. These are simulation checks, not hardware
or throughput evidence. Tests compare control for matching inputs and evaluate
each world's trajectory separately; contact transients differ between engines.

The RTX 3060 Laptop has 6 GB VRAM and still fails the vendor VRAM requirement.
These small camera-free workloads run; this does not establish camera or batch
capacity. Native Storm viewing is available; RTX rendering and cameras remain
separate compatibility work. Local validation includes bimanual CPU/PGS and
GPU/TGS viewing, viewport capture and graceful process shutdown. Both live runs retained all 3,001 headless
state/effort/wrench samples within an absolute tolerance of `1e-7`.

Control is scalar CPU: joint state and effort cross the GPU boundary each step
when GPU physics is selected. URDF-to-USD conversion runs in a child process to
isolate incompatible USD libraries. Mounted sensor bodies and their sensing
joints are included before conversion; device adapters map observations by link
identity. Temporary converted assets are removed after execution.

Isaac Sim is pinned to 6.1.0.0 and PhysX/tensors to 110.3.2. `uv.lock` pins Python
packages, while Kit downloads additional extensions into its cache; this profile
is not offline/hermetic. The native controller uses hidden static-library symbols
to avoid interposition with Kit libraries. Do not replace those linkage settings
with globally exported C++ runtime symbols.

The independent GPU free-fall diagnostic remains under `tools/isaac/`:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python tools/isaac/probe.py
```
