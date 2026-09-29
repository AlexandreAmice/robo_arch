# Isaac dependency profile

This independently locked Python 3.12 profile supplies the external Isaac Sim
runtime without adding its vendor dependencies to the root environment. It runs
the UR7e with Isaac SimulationApp, PhysX with GPU dynamics enabled and the same
Drake inverse-dynamics controller used in the Drake run.
The robot adapter converts the canonical URDF with NVIDIA's USD converter; the
world loop supplies measured joint state and applies effort with drives disabled.

## Run arm tracking

From the repository root:

```sh
uv sync --project third_party/isaac --locked
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python -m robo_arch.scenarios.arm_tracking.run \
  --world isaac --no-sensors --no-browser \
  --record recordings/arm_tracking_isaac.html
```

Open `recordings/arm_tracking_isaac.html` for playback of **measured Isaac joint
positions rendered with Drake geometry**. The adjacent NPZ contains the measured
trace, including the last valid state on a runtime failure. Rendering is separate
from Isaac physics; this does not exercise an Isaac camera. `--no-sensors` is an
explicit change to the scenario, required because camera support is unavailable.
`OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's runtime EULA. Unsetting display
variables prevents a vendor warning dialog from blocking headless startup.

The initial runner supports one fixed-base arm and fixed single-link SDF box
fixtures. Control uses scalar CPU evaluation and copies NumPy state/effort each
step; it is not a tensor-efficient batched implementation. The URDF converter
runs in a child process because its USD libraries conflict with Kit's in one
process. Temporary converted assets are removed after the run.

## Compatibility and measured limits

On September 29, 2026, the 2-second, 2,000-step arm run on an RTX 3060 Laptop
(6 GB, driver 610.57.04; Ubuntu 24.04.4) reached a maximum final joint error of
0.000169 rad. Startup, conversion, simulation and HTML generation took 10.48 s;
peak RSS was 2.58 GiB. Observed GPU memory peaked at 396 MiB with 0.5-second
sampling, from 2 MiB idle. These are one-arm measurements, not batching capacity.

The vendor compatibility checker still rejects this GPU's VRAM. The actual
camera-free workload nevertheless runs. Enabling the native RTX camera extension
failed at extension resolution (`isaacsim.test.docstring` missing), followed by a
vendor teardown crash, before creating a camera. Camera support needs a separate
renderer/Replicator integration; no camera memory-capacity conclusion was obtained.

Isaac Sim is pinned to 6.1.0.0, PhysX/tensors to 110.3.2. `uv.lock` pins Python
packages; Kit downloads further extensions into its user cache, so this profile
is not offline/hermetic. Root dependencies remain independent. The special
OpenCV wheel uses an explicit NVIDIA index. The optional diagnostic under
`tools/isaac/` remains a small independent GPU free-fall check:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  third_party/isaac/.venv/bin/python tools/isaac/probe.py
```

References: [NVIDIA Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html),
[hardware requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html).
