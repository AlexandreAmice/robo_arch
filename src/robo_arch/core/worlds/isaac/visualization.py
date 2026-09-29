"""Observe the native USD/PhysX scene in Kit without changing simulation timing."""

import os
import time

from robo_arch.core.config.worlds import IsaacWorld


def validate_visualization(world: IsaacWorld) -> None:
    """Reject unavailable display intent before starting the native runtime."""
    if world.visualization.mode == "live" and not (
        os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")
    ):
        raise ValueError("Isaac live visualization requires a desktop display")
    if world.visualization.mode == "off" and world.visualization.collision_geometry:
        raise ValueError("Isaac collision display requires visualization.mode: live")


class NativeViewport:
    """Keep Kit observing the exact physics stage; no playback or extra steps.

    This is owned by one SimulationApp. Closing the viewport hides it; quitting
    the app interrupts simulation. USD transforms update at the display cadence.
    """

    def __init__(self, app, stage_id: int, world: IsaacWorld) -> None:
        import carb
        import omni.usd
        from omni.kit.viewport.utility import (
            disable_selection,
            frame_viewport_prims,
            get_active_viewport,
        )
        from omni.physx import get_physx_interface
        from omni.physx.bindings._physx import (
            SETTING_DISPLAY_COLLIDERS,
            SETTING_MOUSE_INTERACTION_ENABLED,
        )

        self._app = app
        self._physics = get_physx_interface()
        self._context = omni.usd.get_context()
        completed = []
        self._context.attach_stage_with_callback(
            stage_id, lambda success, error: completed.append((success, error))
        )
        for _ in range(100):
            app.update()
            if completed:
                break
        if not completed or not completed[0][0]:
            raise RuntimeError(
                f"Isaac viewport could not attach physics stage: {completed}"
            )
        self._viewport = get_active_viewport()
        if self._viewport is None:
            raise RuntimeError(
                "Isaac native viewport extension did not create a viewer"
            )
        settings = carb.settings.get_settings()
        settings.set_bool(SETTING_MOUSE_INTERACTION_ENABLED, False)
        settings.set_int(
            SETTING_DISPLAY_COLLIDERS,
            2 if world.visualization.collision_geometry else 0,
        )
        self._selection_guard = disable_selection(self._viewport)
        frame_viewport_prims(self._viewport, ["/"])

    def update(self) -> None:
        """Publish native PhysX transforms to USD, then render without advancing time."""
        if not self._app.is_running():
            raise RuntimeError("Isaac application was closed during the run")
        self._physics.update_transformations(False, True)
        self._app.update()

    def hold(self) -> None:
        """Keep the final native scene available without advancing physics."""
        while self._app.is_running():
            self._app.update()
            time.sleep(1 / 60)

    def close(self) -> None:
        self._selection_guard = None
        self._context.close_stage()
