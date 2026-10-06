"""Independent nominal dynamics using maintained JaxSim algorithms on CPU/CUDA."""

from dataclasses import dataclass
from functools import partial
from typing import Any

import jax
import jax.numpy as jnp
import jaxsim.api as js
import numpy as np


@dataclass(frozen=True)
class ModelEvaluation:
    """Owned arrays with leading batch axis; forces include gravity and damping."""

    positions: Any
    jacobians: Any
    bias_accelerations: Any
    mass: Any
    bias_force: Any
    acceleration_drift: Any
    acceleration_control: Any
    valid: Any


class NominalModel:
    """One JAX computation, with NumPy and DLPack Torch boundaries.

    The source loader supplies fixed-base trees in q/v/actuator order. JaxSim
    evaluates rigid-body dynamics; explicit viscous damping and diagonal reflected
    rotor inertia are added because its mass/bias query excludes actuation terms.
    """

    def __init__(
        self,
        *,
        native,
        joints,
        velocity_lower,
        velocity_upper,
        limits,
        damping,
        rotor,
        point_bodies,
        points,
        device,
    ):
        jax.config.update("jax_enable_x64", True)
        self.count = len(joints)
        self.joints = joints
        self.velocity_lower = velocity_lower
        self.velocity_upper = velocity_upper
        platform = "gpu" if device.startswith("cuda") else "cpu"
        index = int(device.split(":")[1]) if ":" in device else 0
        self._device = jax.devices(platform)[index]
        self._native = jax.device_put(native, self._device)
        self._order = jnp.asarray([joints.index(j) for j in native.joint_names()])
        self._inverse = jnp.argsort(self._order)
        self._damping = jnp.asarray(damping)
        self._rotor = jnp.asarray(rotor)
        self._indices = tuple(int(i) for i in point_bodies)
        self._points = jnp.asarray(points)
        self._limits = np.array(limits, copy=True)
        self._torch_device = device
        self._evaluate = partial(
            evaluate_batch,
            self._native,
            self._order,
            self._inverse,
            self._damping,
            self._rotor,
            self._indices,
            self._points,
        )

    @property
    def device(self):
        import torch

        return torch.device(self._torch_device)

    @property
    def dtype(self):
        import torch

        return torch.float64

    @property
    def limits(self):
        import torch

        return torch.tensor(self._limits, device=self.device, dtype=self.dtype)

    def evaluate_numpy(self, state: np.ndarray) -> ModelEvaluation:
        """Evaluate owned float64 CPU arrays; no Torch dependency."""
        state = np.asarray(state, dtype=np.float64)
        if state.ndim != 2 or state.shape[1] != 2 * self.count:
            raise ValueError("Dynamics state must have shape [batch, 2*joints]")
        result = self._evaluate(jax.device_put(state, self._device))
        return ModelEvaluation(*(np.array(value, copy=True) for value in result))

    def evaluate(self, state) -> ModelEvaluation:
        """Evaluate Torch float64 [B,2n], sharing device arrays through DLPack."""
        import torch

        if state.ndim != 2 or state.shape[1] != 2 * self.count:
            raise ValueError("Tensor dynamics state must have shape [batch, 2*joints]")
        if state.dtype != self.dtype or state.device != self.device:
            raise ValueError("Tensor dynamics state dtype/device must match its model")
        result = self._evaluate(jax.dlpack.from_dlpack(state.detach().contiguous()))
        return ModelEvaluation(*(torch.from_dlpack(value).clone() for value in result))


@partial(jax.jit, static_argnames=("indices",))
def evaluate_batch(native, order, inverse, damping, rotor, indices, points, states):
    """Compile once per model topology and batch shape, reusing across owners."""

    def positions(q):
        data = js.data.JaxSimModelData.build(native, joint_positions=q[order])
        transforms = js.model.forward_kinematics(native, data)[jnp.asarray(indices)]
        return (
            jnp.einsum("sij,sj->si", transforms[:, :3, :3], points)
            + transforms[:, :3, 3]
        )

    def one(state):
        n = len(order)
        q, v = state[:n], state[n:]
        data = js.data.JaxSimModelData.build(
            native,
            joint_positions=q[order],
            joint_velocities=v[order],
        )
        mass = js.model.free_floating_mass_matrix(native, data)[6:, 6:]
        mass = mass[inverse[:, None], inverse[None, :]] + jnp.diag(rotor)
        bias = (
            js.model.free_floating_bias_forces(native, data)[6:][inverse] + damping * v
        )
        if all(index == 0 for index in indices):
            xyz = points
            jacobian = jnp.zeros((len(indices), 3, n))
            jacobian_rate = jnp.zeros_like(jacobian)
        else:
            xyz = positions(q)
            jacobian = jax.jacfwd(positions)(q)
            jacobian_rate = jax.jvp(jax.jacfwd(positions), (q,), (v,))[1]
        acceleration = jnp.linalg.solve(
            mass, jnp.concatenate((-bias[:, None], jnp.eye(n)), axis=-1)
        )
        valid = jnp.isfinite(state).all() & jnp.isfinite(acceleration).all()
        return (
            xyz,
            jacobian,
            jacobian_rate @ v,
            mass,
            bias,
            acceleration[:, 0],
            acceleration[:, 1:],
            valid,
        )

    return jax.vmap(one)(states)
