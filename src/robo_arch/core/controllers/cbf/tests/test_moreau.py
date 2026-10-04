"""Actual CUDA projections; run with the optional Isaac cbf-gpu profile."""

import numpy as np
import pytest

from robo_arch.core.controllers.cbf.errors import CbfFailure

torch = pytest.importorskip("torch")
pytest.importorskip("moreau")
from robo_arch.core.controllers.cbf.moreau import MoreauProjection  # noqa: E402

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA required")


def tensor(value):
    return torch.as_tensor(value, dtype=torch.float64, device="cuda:0")


def projection(batch=2, count=1, rows=1, limit=2.0):
    return MoreauProjection(
        count=count,
        constraint_count=rows,
        batch_size=batch,
        limits=tensor([limit] * count),
        device="cuda:0",
    )


def test_mixed_batch_and_changed_constraints_stay_on_device(monkeypatch):
    solver = projection()
    coefficient = tensor([[[1.0]], [[1.0]]])
    constant = tensor([[0.0], [0.0]])
    nominal = tensor([[-1.0], [1.0]])

    def reject_host_conversion(*args, **kwargs):
        raise AssertionError("Numerical tensor moved to host")

    monkeypatch.setattr(torch.Tensor, "cpu", reject_host_conversion)
    monkeypatch.setattr(torch.Tensor, "numpy", reject_host_conversion)
    first = solver.solve(coefficient, constant, nominal)
    torch.testing.assert_close(first, tensor([[0.0], [1.0]]), atol=1e-7, rtol=0)
    assert first.is_cuda and solver.last_status == ("Solved", "Solved")
    coefficient.mul_(-1)
    second = solver.solve(coefficient, constant, nominal)
    torch.testing.assert_close(second, tensor([[-1.0], [0.0]]), atol=1e-7, rtol=0)
    # The previous result remains owned and unchanged after another solve.
    torch.testing.assert_close(first, tensor([[0.0], [1.0]]), atol=1e-7, rtol=0)


def test_finite_bounds_and_nominal_passthrough():
    solver = projection(rows=0)
    coefficient = tensor([]).reshape(2, 0, 1)
    constant = tensor([]).reshape(2, 0)
    effort = solver.solve(coefficient, constant, tensor([[-4.0], [0.5]]))
    torch.testing.assert_close(effort, tensor([[-2.0], [0.5]]), atol=1e-7, rtol=0)
    nominal = tensor([[0.2], [-0.8]])
    effort = solver.solve(coefficient, constant, nominal)
    assert torch.equal(effort, nominal) and effort.data_ptr() != nominal.data_ptr()
    assert solver.last_status == ("NominalFeasible", "NominalFeasible")


def test_infeasible_environment_stops_whole_batch():
    solver = projection()
    with pytest.raises(CbfFailure, match="Moreau solve failed") as error:
        solver.solve(
            tensor([[[1.0]], [[1.0]]]), tensor([[0.0], [-3.0]]), tensor([[1.0], [0.0]])
        )
    assert error.value.snapshot["environments"] == [1]
    snapshot = error.value.snapshot
    assert snapshot["coefficient"] == [[[1.0]]]
    assert snapshot["constant"] == [[-3.0]]
    assert snapshot["nominal"] == [[0.0]]
    assert snapshot["limits"] == [2.0]
    assert snapshot["residual_tolerance"] == 1e-6
    # Reproduce just the failing environment from its saved QP data.
    replay = MoreauProjection(
        count=1,
        constraint_count=1,
        batch_size=1,
        limits=tensor(snapshot["limits"]),
        device="cuda:0",
        residual_tolerance=snapshot["residual_tolerance"],
    )
    with pytest.raises(CbfFailure, match="Moreau solve failed"):
        replay.solve(
            tensor(snapshot["coefficient"]),
            tensor(snapshot["constant"]),
            tensor(snapshot["nominal"]),
        )


def test_nonfinite_input_rejected_before_solver():
    solver = projection()
    with pytest.raises(CbfFailure, match="nonfinite") as error:
        solver.solve(
            tensor([[[1.0]], [[float("nan")]]]),
            tensor([[0.0], [0.0]]),
            tensor([[0.0], [0.0]]),
        )
    assert error.value.snapshot["environments"] == [1]


def test_batched_projection_matches_clarabel():
    solvers = pytest.importorskip("pydrake.solvers")
    rng = np.random.default_rng(23)
    batch, rows, count = 4, 9, 3
    coefficient = rng.normal(size=(batch, rows, count))
    constant = rng.uniform(0.1, 0.8, size=(batch, rows))
    nominal = rng.normal(size=(batch, count)) * 4
    solver = projection(batch=batch, count=count, rows=rows)
    effort = solver.solve(tensor(coefficient), tensor(constant), tensor(nominal))
    for index in range(batch):
        program = solvers.MathematicalProgram()
        u = program.NewContinuousVariables(count)
        program.AddQuadraticCost(np.eye(count), -nominal[index], u)
        program.AddBoundingBoxConstraint(-2 * np.ones(count), 2 * np.ones(count), u)
        program.AddLinearConstraint(
            coefficient[index], -constant[index], np.full(rows, np.inf), u
        )
        result = solvers.ClarabelSolver().Solve(program)
        assert result.is_success()
        torch.testing.assert_close(
            effort[index], tensor(result.GetSolution(u)), atol=2e-5, rtol=0
        )


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
