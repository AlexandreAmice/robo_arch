"""Run a camera-free free-fall check in the pinned Isaac Sim Kit runtime."""

import json
import math
import time
from pathlib import Path


def main() -> None:
    """Require real runtime startup and simulated motion; never skip failures."""
    started = time.monotonic()
    import isaacsim  # noqa: F401 -- bootstraps the vendor Kit Python modules
    from kit_app import KitApp

    app = KitApp()
    app.startup([str(Path(__file__).with_suffix(".kit")), "--no-window"])
    try:
        import carb
        from omni.physx import (
            get_physx_simulation_interface,
            get_physx_statistics_interface,
        )
        from omni.physx.bindings._physx import PhysicsSceneStats
        from pxr import (
            Gf,
            PhysicsSchemaTools,
            PhysxSchema,
            Usd,
            UsdGeom,
            UsdPhysics,
            UsdUtils,
        )

        startup_seconds = time.monotonic() - started
        carb.settings.get_settings().set("/physics/updateToUsd", True)
        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        scene = UsdPhysics.Scene.Define(stage, "/physics")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        scene.CreateGravityMagnitudeAttr(9.81)
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physx_scene.CreateEnableGPUDynamicsAttr(True)
        physx_scene.CreateBroadphaseTypeAttr("GPU")
        cube = UsdGeom.Cube.Define(stage, "/cube")
        cube.CreateSizeAttr(0.1)
        cube.AddTranslateOp().Set(Gf.Vec3d(0, 0, 1))
        UsdPhysics.RigidBodyAPI.Apply(cube.GetPrim())
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cache = UsdUtils.StageCache.Get()
        cache.Insert(stage)
        simulation = get_physx_simulation_interface()
        simulation.attach_stage(cache.GetId(stage).ToLongInt())
        try:
            step_seconds = 1 / 120
            steps = 30
            for index in range(steps):
                simulation.simulate(step_seconds, index * step_seconds)
                simulation.fetch_results()
            final_z_m = float(cube.GetPrim().GetAttribute("xformOp:translate").Get()[2])
            expected_z_m = 1 - 9.81 * step_seconds**2 * steps * (steps + 1) / 2
            if not math.isfinite(final_z_m) or abs(final_z_m - expected_z_m) > 1e-5:
                raise RuntimeError(f"Free-fall check failed: z={final_z_m} m")
            stats = PhysicsSceneStats()
            valid_stats = get_physx_statistics_interface().get_physx_scene_statistics(
                cache.GetId(stage).ToLongInt(),
                PhysicsSchemaTools.sdfPathToInt(scene.GetPath()),
                stats,
            )
            if not valid_stats or stats.gpu_mem_heap_solver == 0:
                raise RuntimeError("No PhysX GPU solver allocation reported")
            print(
                json.dumps(
                    {
                        "check": "free_fall",
                        "steps": steps,
                        "step_seconds": step_seconds,
                        "final_z_m": final_z_m,
                        "semi_implicit_euler_z_m": expected_z_m,
                        "startup_seconds": startup_seconds,
                        "gpu_dynamics_requested": True,
                        "gpu_heap_bytes": stats.gpu_mem_heap,
                        "gpu_solver_heap_bytes": stats.gpu_mem_heap_solver,
                    }
                ),
                flush=True,
            )
        finally:
            simulation.detach_stage()
            cache.Erase(stage)
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
