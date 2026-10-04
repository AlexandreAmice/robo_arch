"""Native Isaac viewport observing sampled state from the selected backend."""

import asyncio
import signal
import time
from pathlib import Path

from robo_arch.core.worlds.isaac.config import IsaacVisualization


class Viewer:
    """Own the observational camera, frame cadence and viewport capture lifecycle."""

    def __init__(self, app, scene, config: IsaacVisualization) -> None:
        import omni.kit.app
        import omni.timeline
        import omni.usd
        from carb.eventdispatcher import get_eventdispatcher
        from omni.kit.viewport.utility import get_active_viewport
        from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

        self.scene = scene
        stage = scene.stage
        self._body_transforms = None
        self.app = app
        self.config = config
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        timeline.set_auto_update(False)
        self.context = omni.usd.get_context()
        self.stage = stage
        # The converter marks collider meshes as guides. Hide only their
        # illustration in the session layer; their physics APIs stay active.
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            for prim in stage.Traverse():
                if prim.HasAPI(UsdPhysics.CollisionAPI):
                    drawable = UsdGeom.Imageable(prim)
                    if drawable.ComputePurpose() == UsdGeom.Tokens.guide:
                        drawable.MakeInvisible()
        self.closed = False
        self._kit = omni.kit.app.get_app()
        self._quit_subscription = None
        if self.context.get_stage() != stage:
            success, error = self._wait(self.context.attach_stage_async(stage))
            if not success:
                raise RuntimeError(f"Could not attach Isaac viewport stage: {error}")
        self.viewport = get_active_viewport()
        if self.viewport is None:
            raise RuntimeError("Isaac did not create a native viewport")
        self.camera = UsdGeom.Camera.Define(stage, "/inspection/camera")
        self.camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000))
        self.camera.CreateFocalLengthAttr(30)
        self._camera_transform = self.camera.AddTransformOp()
        self._frame_scene()
        self._framed = False
        light = UsdLux.DomeLight.Define(stage, "/inspection/light")
        light.CreateIntensityAttr(1000)
        self.viewport.camera_path = self.camera.GetPath()
        self.viewport.resolution = (config.width, config.height)
        self.next_frame = 0.0
        self.wall_start = time.monotonic()
        for _ in range(10):
            app.update()
        self._quit_subscription = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_POST_QUIT,
            on_event=self._request_close,
            observer_name="robo_arch viewport cleanup",
        )

    def _frame_scene(self) -> None:
        from pxr import Gf, Usd, UsdGeom

        bounds = (
            UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            )
            .ComputeWorldBound(self.stage.GetPseudoRoot())
            .ComputeAlignedRange()
        )
        if bounds.IsEmpty():
            raise ValueError("Isaac viewport has no visible geometry to frame")
        center = bounds.GetMidpoint()
        radius = max(bounds.GetSize().GetLength() / 2, 0.5)
        self._camera_transform.Set(
            Gf.Matrix4d()
            .SetLookAt(
                center + radius * Gf.Vec3d(1.8, -2.2, 1.4),
                center,
                Gf.Vec3d(0, 0, 1),
            )
            .GetInverse()
        )

    def _request_close(self, event) -> None:
        # Keep Kit alive until the caller saves traces and detaches physics.
        self._kit.try_cancel_shutdown("robo_arch is cleaning up its physics stage")
        self.closed = True

    def _wait(self, coroutine, timeout: float = 60.0):
        task = asyncio.ensure_future(coroutine)
        deadline = time.monotonic() + timeout
        try:
            while not task.done():
                if time.monotonic() >= deadline:
                    raise TimeoutError("Isaac viewport operation timed out")
                self.app.update()
            return task.result()
        finally:
            if not task.done():
                task.cancel()

    def _publish(self) -> None:
        if self.scene.world.physics.backend == "newton":
            self._publish_newton()
            return
        from omni.physx import get_physx_interface

        # Control samples every physics step; USD transforms need only be copied
        # at the display cadence. Writing USD at 1 kHz stalls the scene delegate.
        get_physx_interface().update_transformations(False, True)

    def _publish_newton(self) -> None:
        """Publish sampled link poses to USD for Storm (which does not use Fabric).

        This CPU copy occurs at display cadence only. Physics reads native state.
        """
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            if self._body_transforms is None:
                self._body_transforms = []
                for name, arm in self.scene.native.articulations.items():
                    body_ids = {name: i for i, name in enumerate(arm.body_names)}
                    for env in range(self.scene.world.num_envs):
                        root = self.stage.GetPrimAtPath(f"/World/envs/env_{env}/{name}")
                        for prim in Usd.PrimRange(root):
                            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                                body = body_ids[prim.GetName()]
                                transform = UsdGeom.Xformable(prim).MakeMatrixXform()
                                self._body_transforms.append(
                                    (name, env, body, prim, transform)
                                )
            poses = {
                name: arm.data.body_link_pose_w.torch.cpu().numpy()
                for name, arm in self.scene.native.articulations.items()
            }
            cache = UsdGeom.XformCache()
            for name, env, body, prim, transform in self._body_transforms:
                pose = poses[name][env, body].astype(float)
                matrix = Gf.Matrix4d(1)
                matrix.SetRotate(Gf.Quatd(pose[6], Gf.Vec3d(*pose[3:6])))
                matrix.SetTranslateOnly(Gf.Vec3d(*pose[:3]))
                parent = cache.GetLocalToWorldTransform(prim.GetParent())
                transform.Set(matrix * parent.GetInverse())
                cache.Clear()

    def update(self, t: float, realtime_rate: float) -> None:
        """Render sampled states without advancing the physics or its timeline."""
        if t + 1e-12 < self.next_frame:
            return
        self._publish()
        if not self._framed:
            self._frame_scene()
            self._framed = True
        self.app.update()
        if self.closed or not self.app.is_running():
            raise RuntimeError("Isaac window closed before the run completed")
        self.next_frame = t + self.config.publish_period
        if realtime_rate > 0:
            delay = self.wall_start + t / realtime_rate - time.monotonic()
            if delay > 0:
                time.sleep(delay)

    def capture(self, destination: Path) -> None:
        """Save the native final viewport, waiting for image writing to finish."""
        import omni.kit.renderer_capture
        from omni.kit.viewport.utility import capture_viewport_to_file

        destination.parent.mkdir(parents=True, exist_ok=True)
        self._publish()
        for _ in range(10):
            self.app.update()
        capture = capture_viewport_to_file(self.viewport, str(destination.resolve()))
        result = self._wait(capture.wait_for_result())
        # The viewport future only schedules the asynchronous file write. Join
        # the writer before checking the file or unloading renderer extensions.
        omni.kit.renderer_capture.acquire_renderer_capture_interface().wait_async_capture()
        if not result:
            raise RuntimeError("Isaac viewport produced no color image")
        if not destination.is_file():
            raise RuntimeError(f"Isaac did not write the viewport image: {destination}")

    def hold(self) -> None:
        """Retain the final native scene for camera navigation."""
        print(
            "Isaac run complete. Close the window or press Ctrl-C to finish.",
            flush=True,
        )

        def finish(signum, frame):
            self.closed = True

        handler = signal.signal(signal.SIGINT, finish)
        try:
            while not self.closed and self.app.is_running():
                self.app.update()
                time.sleep(0.01)
        finally:
            signal.signal(signal.SIGINT, handler)

    def pause_rendering(self) -> None:
        """Stop viewport drawing and drain work before native stage teardown."""
        import omni.appwindow
        import omni.kit.renderer.bind

        self.viewport.updates_enabled = False
        renderer = omni.kit.renderer.bind.get_renderer_interface()
        renderer.wait_idle(omni.appwindow.get_default_app_window())

    def close(self, *, close_stage: bool = True) -> None:
        import omni.appwindow
        import omni.kit.renderer.bind

        self.pause_rendering()
        renderer = omni.kit.renderer.bind.get_renderer_interface()
        window = omni.appwindow.get_default_app_window()
        try:
            # Image writing and GPU rendering have separate completion fences.
            # Drain drawing before destroying its stage or unloading Kit.
            renderer.wait_idle(window)
            if close_stage:
                success, error = self._wait(self.context.close_stage_async())
                if not success:
                    raise RuntimeError(f"Could not close Isaac viewport stage: {error}")
            renderer.wait_idle(window)
        finally:
            self._quit_subscription = None
            self._camera_transform = None
            self.camera = None
            self._body_transforms = None
            self.scene = None
            self.stage = None
            self.viewport = None
            self.context = None
            self._kit = None
