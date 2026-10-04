"""CUDA buffer/reset behavior and native USD clone isolation without Kit startup."""

from types import SimpleNamespace

import numpy as np
import pytest

from robo_arch.core.worlds.isaac.batching import TorchArticulations, clone_environments
from robo_arch.core.worlds.isaac.config import IsaacWorld
from robo_arch.core.worlds.isaac.visualization import (
    make_display_stage,
    publish_link_poses,
)


@pytest.mark.parametrize(
    "settings",
    [
        {"control_backend": "numpy", "batch_size": 2},
        {"control_backend": "torch", "physics": {"device": "cpu"}},
        {"batch_size": 0},
        {"log_every_n_steps": 0},
    ],
)
def test_invalid_batch_settings_fail_without_sdk(settings):
    with pytest.raises(ValueError):
        IsaacWorld.model_validate(settings)


def test_clone_world_anchors_relationships_and_collision_groups():
    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    UsdPhysics = pytest.importorskip("pxr.UsdPhysics")
    Sdf = pytest.importorskip("pxr.Sdf")
    stage = Usd.Stage.CreateInMemory()
    source = "/_environments/env_000000"
    UsdGeom.Xform.Define(stage, source)
    body = UsdGeom.Xform.Define(stage, source + "/arm/body")
    joint = UsdPhysics.FixedJoint.Define(stage, source + "/arm/anchor")
    joint.CreateBody1Rel().AddTarget(body.GetPath())
    joint.CreateLocalPos0Attr().Set((0.3, 0.2, 0.1))
    physics = UsdPhysics.Scene.Define(stage, "/physics")
    scene = SimpleNamespace(
        stage=stage,
        physics=SimpleNamespace(
            CreateInvertCollisionGroupFilterAttr=lambda value: (
                physics.GetPrim()
                .CreateAttribute("test:inverted", Sdf.ValueTypeNames.Bool)
                .Set(value)
            )
        ),
    )
    config = IsaacWorld(control_backend="torch", batch_size=3, environment_spacing=2)
    clone_environments(scene, config)
    np.testing.assert_allclose(
        scene.environment_origins, [[0, 0, 0], [2, 0, 0], [0, 2, 0]]
    )
    for i, (path, offset) in enumerate(
        zip(scene.environment_paths, scene.environment_origins, strict=True)
    ):
        anchor = UsdPhysics.Joint.Get(stage, path + "/arm/anchor")
        np.testing.assert_allclose(
            anchor.GetLocalPos0Attr().Get(), offset + [0.3, 0.2, 0.1], atol=1e-7
        )
        assert anchor.GetBody1Rel().GetTargets() == [Sdf.Path(path + "/arm/body")]
        group = UsdPhysics.CollisionGroup.Get(stage, f"/_collisions/env_{i:06d}")
        assert group.GetCollidersCollectionAPI().GetIncludesRel().GetTargets() == [
            Sdf.Path(path)
        ]
        assert set(group.GetFilteredGroupsRel().GetTargets()) == {
            group.GetPath(),
            Sdf.Path("/_collisions/ground"),
        }


def test_gpu_viewport_pose_edits_do_not_change_physics_stage():
    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    UsdPhysics = pytest.importorskip("pxr.UsdPhysics")
    source = Usd.Stage.CreateInMemory()
    UsdPhysics.Scene.Define(source, "/physics")
    UsdGeom.Xform.Define(source, "/root").AddTranslateOp().Set((3, 0, 0))
    body = UsdGeom.Xform.Define(source, "/root/body")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    UsdPhysics.CollisionAPI.Apply(body.GetPrim())
    UsdGeom.Xform.Define(source, "/root/body/child")
    joint = UsdPhysics.FixedJoint.Define(source, "/anchor")
    joint.CreateBody1Rel().AddTarget(body.GetPath())
    display = make_display_stage(source)
    assert display.GetRootLayer() == source.GetRootLayer()
    assert display.GetSessionLayer() != source.GetSessionLayer()
    UsdGeom.Camera.Define(display, "/inspection/camera")
    assert not source.GetPrimAtPath("/inspection/camera")
    poses = {
        "/root/body/child": [6, 3, 0, 0, 0, 0, 1],
        "/root/body": [5, 2, 0, 0, 0, 0, 1],
    }
    publish_link_poses(display, poses)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    for path, pose in poses.items():
        np.testing.assert_allclose(
            cache.GetLocalToWorldTransform(
                display.GetPrimAtPath(path)
            ).ExtractTranslation(),
            pose[:3],
        )
    np.testing.assert_allclose(
        cache.GetLocalToWorldTransform(body.GetPrim()).ExtractTranslation(), [3, 0, 0]
    )
    assert body.GetPrim().HasAPI(UsdPhysics.RigidBodyAPI)
    assert not display.GetPrimAtPath("/root/body").HasAPI(UsdPhysics.RigidBodyAPI)
    assert not display.GetPrimAtPath("/root/body").HasAPI(UsdPhysics.CollisionAPI)
    assert source.GetPrimAtPath("/physics").IsActive()
    assert not display.GetPrimAtPath("/physics").IsActive()
    assert source.GetPrimAtPath("/anchor").IsActive()
    assert not display.GetPrimAtPath("/anchor").IsActive()


@pytest.fixture
def runtime():
    torch = pytest.importorskip("torch")
    if not torch.cuda.is_available():
        pytest.skip("Requires CUDA; this adapter has no CPU fallback")
    paths = ("/_environments/env_000000", "/_environments/env_000001")

    class Arm:
        count = 2
        shared_metatype = SimpleNamespace(fixed_base=True, dof_names=("joint",))
        prim_paths = [path + "/arm/anchor" for path in paths]

        def __init__(self):
            self.q = torch.zeros((2, 1), device="cuda:0")
            self.v = self.q.clone()
            self.u = self.q.clone()

        def get_dof_limits(self):
            return torch.tensor([[[-3.0, 3.0]]] * 2)

        def get_dof_positions(self):
            return self.q

        def get_dof_velocities(self):
            return self.v

        def set_dof_positions(self, data, indices):
            self.q[indices] = data[indices]

        def set_dof_velocities(self, data, indices):
            self.v[indices] = data[indices]

        def set_dof_actuation_forces(self, data, indices):
            self.u[indices] = data[indices]

    arm = Arm()
    view = SimpleNamespace(
        device="cuda:0",
        create_articulation_view=lambda paths: arm,
        invalidate=lambda: None,
    )
    scene = SimpleNamespace(
        environment_paths=paths,
        roots={"arm": paths[0] + "/arm/anchor"},
        devices=SimpleNamespace(
            robots=(SimpleNamespace(name="arm", model="model", initial_positions=None),)
        ),
        definitions=SimpleNamespace(
            robots={
                "model": SimpleNamespace(joints=("joint",), default_positions=(0.2,))
            }
        ),
    )
    return TorchArticulations(scene, view)


def test_cuda_commands_and_independent_reset(runtime):
    import torch

    arm = runtime.arms["arm"]
    arm.q[:] = torch.tensor([[0.6], [0.9]], device="cuda:0")
    arm.v.fill_(0.4)
    runtime.times[:] = torch.tensor([1.0, 2.0], device="cuda:0")

    class Policy:
        def __call__(self, state, times):
            assert state.device.type == times.device.type == "cuda"
            assert state.dtype == times.dtype == torch.float64
            self.diagnostics = {"test/clearance": state[:, :1]}
            return state[:, :1] + times[:, None]

    policy = Policy()
    runtime.apply({"arm": policy})
    torch.testing.assert_close(arm.u, torch.tensor([[1.6], [2.9]], device="cuda:0"))
    runtime.reset(torch.tensor([1], device="cuda:0", dtype=torch.int32))
    torch.testing.assert_close(arm.q, torch.tensor([[0.6], [0.2]], device="cuda:0"))
    torch.testing.assert_close(arm.v, torch.tensor([[0.4], [0.0]], device="cuda:0"))
    torch.testing.assert_close(arm.u, torch.tensor([[1.6], [0.0]], device="cuda:0"))
    torch.testing.assert_close(
        runtime.times, torch.tensor([1.0, 0.0], device="cuda:0", dtype=torch.float64)
    )
    runtime.apply({"arm": policy})
    sample = runtime.sample({"arm": policy})
    np.testing.assert_allclose(sample["test/clearance"], [[0.6], [0.2]], atol=1e-7)
    with pytest.raises(ValueError, match="unique"):
        runtime.reset(torch.tensor([1, 1], device="cuda:0", dtype=torch.int32))


def test_cuda_adapter_rejects_host_command_and_nonfinite_state(runtime):
    import torch

    with pytest.raises(ValueError, match="invalid CUDA effort"):
        runtime.apply({"arm": lambda state, times: torch.zeros((2, 1))})
    runtime.arms["arm"].q[1] = float("nan")
    with pytest.raises(RuntimeError, match="invalid CUDA state"):
        runtime.apply({"arm": lambda state, times: state[:, :1]})


def test_closed_runtime_cannot_access_unloaded_native_views(runtime):
    arm = runtime.arms["arm"]
    view = runtime.view
    runtime.close()
    runtime.close()
    assert arm._backend is None and view._backend is None
    with pytest.raises(RuntimeError, match="closed"):
        runtime.apply({})
    with pytest.raises(RuntimeError, match="closed"):
        runtime.reset(runtime.indices)
    with pytest.raises(RuntimeError, match="closed"):
        runtime.sample({})


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
