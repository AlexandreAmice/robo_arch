# Local Isaac environment

This independent uv project runs a small, camera-free physics diagnostic using
Isaac-distributed Kit/PhysX. It does not install or exercise the full Isaac
Core/SimulationApp API, robots or controllers.

On the development laptop, the diagnostic runs but NVIDIA's compatibility
checker rejects the 6 GB GPU. This establishes that the small physics case runs;
it does not establish capacity or support for robot scenes, cameras or batching.

## Run the diagnostic

From the repository root, using the project's pinned uv version:

```sh
cd deployment/isaac
uv sync --locked
mkdir -p .cache
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  PYTHONUNBUFFERED=1 timeout --signal=TERM --kill-after=10s 180s \
  /usr/bin/time -v .venv/bin/python probe.py > .cache/probe.log 2>&1
```

`OMNI_KIT_ACCEPT_EULA=YES` accepts NVIDIA's runtime EULA. Unsetting display
variables prevents a vendor IOMMU warning dialog from blocking headless startup;
it does not disable IOMMU. The physics probe raises on missing runtime, incorrect
motion or absent GPU solver allocation.

Run the vendor compatibility checker separately:

```sh
env -u DISPLAY -u WAYLAND_DISPLAY OMNI_KIT_ACCEPT_EULA=YES \
  PYTHONUNBUFFERED=1 timeout --signal=TERM --kill-after=10s 120s \
  /usr/bin/time -v .venv/bin/isaacsim isaacsim.exp.compatibility_check \
  --no-window --/app/quitAfter=10 --/app/settings/persistent=false \
  --/privacy/userConsent=false > .cache/compatibility.log 2>&1
```

Read the reported result: the vendor checker can exit **0 even when checks
fail**. `packaging==26.0` supplies an undeclared import dependency needed by that
checker.

## Dependencies and compatibility

Python is pinned to 3.12.13, Isaac Sim to 6.1.0.0, and direct Kit extension
dependencies in `probe.kit`. The local `.venv` and lock do not alter the root
project environment. Kit downloads additional extensions into its user cache;
those transitive artifacts are not covered by `uv.lock`, so this profile is not
an offline/hermetic installation.

Observed on September 28, 2026:

| Item | Result |
|---|---|
| Platform | Ubuntu 24.04.4, x86-64, glibc 2.39 |
| GPU / driver | RTX 3060 Laptop, 6144 MiB VRAM, NVIDIA 610.57.04 |
| Runtime | Kit 110.3.0 (`00c488ae`), PhysX extension 110.3.2 |
| Vendor checker | Insufficient VRAM; driver, RTX and OS checks pass |
| Physics | 30 steps at 1/120 s; cube falls from 1 m to 0.6832187175750732 m |
| Numerical agreement | Within 1e-5 m of semi-implicit Euler's 0.68321875 m |
| GPU execution | Nonzero GPU solver allocation reported by PhysX scene statistics |

The checker uses a 10 GB VRAM threshold; the published requirements checked at
that time list 16 GB. This laptop fails both. Full API startup and the intended
robot workload need their own validation before this environment can support
controller evaluation.

References: [NVIDIA requirements](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/requirements.html),
[Python installation](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/install_python.html),
[direct PhysX simulation](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/simulation_control/simulation_control.html).
