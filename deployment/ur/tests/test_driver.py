"""Actual UR driver/ros2_control mock process tests; missing ROS fails collection."""

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import pytest
import rclpy  # noqa: F401 -- requested provider must exist at collection

from robo_arch.core.config.declarations import CalibrationProfile, DeviceBinding, Pose
from robo_arch.core.contracts.commands import CommandKind
from robo_arch.core.worlds.assembly import PlacedRobot
from robo_arch.core.worlds.real.config import RealWorld
from robo_arch.core.worlds.real.trajectory import TrajectoryPoint
from robo_arch.robots.ur7e.real import UrTrajectoryAdapter


@pytest.fixture(scope="module")
def driver(tmp_path_factory):
    log_path = tmp_path_factory.mktemp("driver") / "driver.log"
    with log_path.open("w") as log:
        process = subprocess.Popen(
            [
                "ros2",
                "launch",
                "ur_robot_driver",
                "ur_control.launch.py",
                "ur_type:=ur7e",
                "robot_ip:=127.0.0.1",
                "use_mock_hardware:=true",
                "initial_joint_controller:=joint_trajectory_controller",
                "launch_rviz:=false",
            ],
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            yield process
        finally:
            os.killpg(process.pid, signal.SIGCONT)
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=15)
            print(log_path.read_text())


@pytest.fixture
def client(driver):
    robot = PlacedRobot(
        name="cell/arm",
        model="ur7e",
        poses=(Pose(),),
        initial_positions=None,
        binding=DeviceBinding(identity="synthetic-test-arm", endpoint="/"),
        mounting_revision="mock-base-v1",
        calibration=CalibrationProfile(
            identity="synthetic-test-arm",
            parent_frame="world",
            child_frame="base_link",
            mounting_revision="mock-base-v1",
            kind="synthetic",
            pose=Pose(),
        ),
    )
    adapter = UrTrajectoryAdapter(
        robot,
        RealWorld(operation_timeout=15, observation_max_age=0.3),
        CommandKind.JOINT_POSITION_TRAJECTORY,
    )
    try:
        adapter.connect()
        yield adapter
    finally:
        adapter.close()


def test_installed_package():
    import robo_arch

    assert Path(robo_arch.__file__).is_relative_to("/opt/robo")


def test_trajectory_state_roundtrip_and_reset(client):
    initial = client.observation()
    target = list(initial.positions)
    target[0] += 0.1
    client.send((TrajectoryPoint(tuple(target), 1.0),))
    trace = []
    for _ in range(70):
        client.spin(0.02)
        sample = client.observation()
        trace.append({"stamp": sample.stamp, "positions": sample.positions})
        time.sleep(0.02)
    client.finish()
    assert client.observation().positions == pytest.approx(target, abs=0.001)
    assert client.instance.binding.identity == "synthetic-test-arm"
    assert client.instance.calibration.kind == "synthetic"
    before_reset = client.observation().positions
    assert client.reset().positions == pytest.approx(before_reset, abs=0.001)
    output = Path(os.environ["ROBO_TRACE_DIR"])
    (output / "trajectory.json").write_text(
        json.dumps(
            {
                "source": "UR ros2_control mock hardware; not physical motion",
                "identity": client.instance.binding.identity,
                "joint_names": client.names,
                "initial": initial.positions,
                "target": target,
                "observations": trace,
            },
            indent=2,
        )
    )


def test_cancel_and_reconnect_never_replay(client):
    target = list(client.observation().positions)
    target[0] += 0.2
    client.send((TrajectoryPoint(tuple(target), 10.0),))
    client.cancel()
    client.disconnect()
    with pytest.raises(ConnectionError):
        client.send((TrajectoryPoint(tuple(target), 1.0),))
    sample = client.connect()
    assert abs(sample.positions[0] - target[0]) > 0.1
    client.spin(0.1)
    assert client.observation().positions[0] == pytest.approx(
        sample.positions[0], abs=0.002
    )


def test_stalled_process_invalidates_state_and_recovers(client, driver):
    os.killpg(driver.pid, signal.SIGSTOP)
    try:
        time.sleep(0.4)
        with pytest.raises(TimeoutError, match="Stale"):
            client.observation()
        with pytest.raises(ConnectionError):
            client.send((TrajectoryPoint((0.0,) * 6, 1.0),))
    finally:
        os.killpg(driver.pid, signal.SIGCONT)
    for _ in range(20):
        client.spin(0.01)
    assert len(client.connect().positions) == 6


def test_timed_out_goal_is_settled_and_canceled_on_reconnect(client, driver):
    target = list(client.observation().positions)
    target[0] += 0.2
    client.timeout = 0.15
    os.killpg(driver.pid, signal.SIGSTOP)
    try:
        with pytest.raises(TimeoutError):
            client.send((TrajectoryPoint(tuple(target), 10.0),))
        assert client._pending_goal is not None
    finally:
        os.killpg(driver.pid, signal.SIGCONT)
        client.timeout = 15
    client.connect()
    assert client._pending_goal is None and client._goal is None
    assert abs(client.observation().positions[0] - target[0]) > 0.1


def test_inactive_controller_rejects_goal(client):
    from controller_manager_msgs.srv import SwitchController

    service = client.node.create_client(
        SwitchController, "/controller_manager/switch_controller"
    )
    assert service.wait_for_service(timeout_sec=5)
    request = SwitchController.Request()
    request.strictness = SwitchController.Request.STRICT
    request.deactivate_controllers = ["joint_trajectory_controller"]
    assert client._wait(service.call_async(request)).ok
    try:
        with pytest.raises(RuntimeError, match="rejected"):
            client.send((TrajectoryPoint(client.observation().positions, 1.0),))
    finally:
        request.deactivate_controllers = []
        request.activate_controllers = ["joint_trajectory_controller"]
        assert client._wait(service.call_async(request)).ok
        client.node.destroy_client(service)
