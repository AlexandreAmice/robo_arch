"""GPU task state and masked independent reaching episodes."""

import torch
from torch import Tensor

from robo_arch.scenarios.batched_reaching.config import Reaching


class Episodes:
    """Own targets, clocks, completion masks and statistics on one device."""

    def __init__(self, initial: Tensor, limits: Tensor, config: Reaching) -> None:
        self.config = config
        self.initial = initial
        self.low = torch.maximum(initial - config.target_radius, limits[..., 0])
        self.high = torch.minimum(initial + config.target_radius, limits[..., 1])
        self.generator = torch.Generator(device=initial.device).manual_seed(config.seed)
        self.targets = initial.clone()
        self.age = torch.zeros(len(initial), device=initial.device)
        self.settled = torch.zeros_like(self.age)
        self.successes = torch.zeros_like(self.age, dtype=torch.int64)
        self.timeouts = torch.zeros_like(self.successes)
        self.error_sum = torch.zeros_like(self.age)
        self.completed_error_sum = torch.zeros_like(self.age)
        self.max_error = torch.zeros_like(self.age)
        self.invalid = torch.zeros_like(self.age, dtype=torch.bool)
        self.pending_success = torch.zeros_like(self.invalid)
        self.pending_timeout = torch.zeros_like(self.invalid)
        self.steps = 0
        self.reset(torch.ones_like(self.invalid))

    def reset(self, mask: Tensor) -> None:
        """Sample independent goals and clear only selected episode state."""
        random = torch.rand(
            self.targets.shape, generator=self.generator, device=self.targets.device
        )
        goals = self.low + random * (self.high - self.low)
        self.targets.copy_(torch.where(mask[:, None], goals, self.targets))
        self.age.masked_fill_(mask, 0)
        self.settled.masked_fill_(mask, 0)
        self.pending_success.masked_fill_(mask, False)
        self.pending_timeout.masked_fill_(mask, False)

    def observe(self, q: Tensor, v: Tensor, dt: float) -> Tensor:
        error = (q - self.targets).abs().amax(dim=-1)
        self.invalid |= ~torch.isfinite(q).all(dim=-1) | ~torch.isfinite(v).all(dim=-1)
        active = ~(self.pending_success | self.pending_timeout)
        self.age += active * dt
        reached = (error <= self.config.tolerance) & (
            v.abs().amax(dim=-1) <= self.config.velocity_tolerance
        )
        self.settled.copy_(torch.where(reached, self.settled + dt, 0.0))
        success = active & (self.settled >= self.config.settle_seconds)
        timeout = active & ~success & (self.age >= self.config.episode_seconds)
        self.successes += success
        self.timeouts += timeout
        self.completed_error_sum += torch.where(success | timeout, error, 0.0)
        self.pending_success |= success
        self.pending_timeout |= timeout
        self.error_sum += error
        self.max_error = torch.maximum(self.max_error, error)
        self.steps += 1
        return self.pending_success | self.pending_timeout
