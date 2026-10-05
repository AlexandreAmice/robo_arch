import numpy as np

from robo_arch.examples.cbf_filter import DEFAULT_RUN as CBF_RUN
from robo_arch.examples.cbf_filter import evaluate
from robo_arch.examples.drake_scene import DEFAULT_RUN as SCENE_RUN
from robo_arch.examples.drake_scene import describe


def test_drake_scene_builds_without_running_a_simulator() -> None:
    text = describe(SCENE_RUN)

    assert "world: drake" in text
    assert "robots: arm" in text
    assert "controller_models: arm" in text
    assert "cameras: camera" in text
    assert "objects: box" in text


def test_cbf_example_returns_an_accepted_command() -> None:
    summary = evaluate(CBF_RUN)

    assert summary.geometry_constraints > 0
    assert summary.velocity_constraints > 0
    assert summary.minimum_clearance_m >= 0
    assert np.isfinite(summary.effort_correction_norm_Nm)
