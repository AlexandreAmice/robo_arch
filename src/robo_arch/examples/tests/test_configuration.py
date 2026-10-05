from robo_arch.examples.configuration import DEFAULT_RUN, describe, main


def test_nested_configuration_is_visible_without_a_world_sdk(capsys) -> None:
    text = describe(DEFAULT_RUN)

    assert "world: drake" in text
    assert "left_arm: ur7e" in text
    assert "right/arm: iiwa7" in text
    assert "right/wrist_ft: ati_mini45 -> right/arm/iiwa_link_ee" in text
    assert "left_ft: ati_mini45 -> left_arm/tool0" in text

    assert main([]) == 0
    assert "left_arm: ur7e" in capsys.readouterr().out


def test_an_alternate_run_shows_declared_objects(capsys) -> None:
    reference = "package://robo_arch/scenarios/arm_tracking/scenario.yaml"
    text = describe(reference)

    assert "arm: ur7e" in text
    assert "camera: realsense_d435 -> arm/tool0" in text
    assert "box: box" in text

    assert main(["--run", reference]) == 0
    assert "box: box" in capsys.readouterr().out
