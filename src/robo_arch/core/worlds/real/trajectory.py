"""ROS 2 joint trajectory action transport, imported only for deployment.

The client owns its node/context and spins synchronously in the caller thread.
Observations are named positions/velocities, never inferred joint efforts.
Connection loss invalidates local state; it cannot guarantee a remote stop.
"""

import math
import re
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class JointObservation:
    """Joint state in declared order; radians, radians/s, ROS seconds."""

    names: tuple[str, ...]
    positions: tuple[float, ...]
    velocities: tuple[float, ...]
    stamp: float
    received: float


@dataclass(frozen=True)
class TrajectoryPoint:
    """Joint positions/rates and positive seconds from the trajectory start."""

    positions: tuple[float, ...]
    time_from_start: float
    velocities: tuple[float, ...] = ()


class TrajectoryTransport:
    """Own a FollowJointTrajectory client and a fresh JointState subscription.

    ``namespace`` identifies a ROS endpoint, not a physical robot. Names must be
    the selected driver's exact joint names. Wall-clock time bounds waits even
    if a ROS simulation clock stops. ``max_age`` bounds both source timestamps
    and receipt age; reconnect/reset requires a subsequently received sample.
    """

    def __init__(
        self,
        *,
        names: tuple[str, ...],
        namespace: str,
        controller: str = "joint_trajectory_controller",
        state_topic: str = "joint_states",
        max_age: float = 0.5,
        timeout: float = 5.0,
    ) -> None:
        if not names or len(set(names)) != len(names):
            raise ValueError("Joint names must be nonempty and unique")
        if re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/?)*", namespace) is None:
            raise ValueError(
                "Use an absolute ROS namespace for the installation endpoint"
            )
        for endpoint in (controller, state_topic):
            if (
                re.fullmatch(
                    r"[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*", endpoint
                )
                is None
            ):
                raise ValueError(
                    "Controller/topic names must be relative to the namespace"
                )
        if (
            not math.isfinite(max_age)
            or max_age <= 0
            or not math.isfinite(timeout)
            or timeout <= 0
        ):
            raise ValueError("Age and timeout must be finite and positive")
        from control_msgs.action import FollowJointTrajectory
        from rclpy.action import ActionClient
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import JointState

        self.names = names
        self.max_age = max_age
        self.timeout = timeout
        self.context = Context()
        self.context.init()
        self.node = Node(
            "robo_arch_trajectory", namespace=namespace, context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.client = ActionClient(
            self.node, FollowJointTrajectory, f"{controller}/follow_joint_trajectory"
        )
        self.subscription = self.node.create_subscription(
            JointState, state_topic, self._observe, qos_profile_sensor_data
        )
        self._sample: JointObservation | None = None
        self._goal = None
        self._pending_goal = None
        self._result = None
        self._connected = False

    def _observe(self, message) -> None:
        names = tuple(message.name)
        if len(set(names)) != len(names) or not set(self.names) <= set(names):
            raise ValueError("JointState has duplicate or missing required joint names")
        if len(message.position) != len(names) or len(message.velocity) != len(names):
            raise ValueError(
                "JointState must provide positions and velocities for all names"
            )
        indices = [names.index(name) for name in self.names]
        positions = tuple(float(message.position[index]) for index in indices)
        velocities = tuple(float(message.velocity[index]) for index in indices)
        if not all(math.isfinite(value) for value in (*positions, *velocities)):
            raise ValueError("JointState contains nonfinite values")
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        # Delayed/out-of-order messages cannot replace newer observations.
        if self._sample is None or stamp > self._sample.stamp:
            self._sample = JointObservation(
                self.names, positions, velocities, stamp, time.monotonic()
            )

    def spin(self, timeout: float = 0.01) -> None:
        self.executor.spin_once(timeout_sec=timeout)

    def _wait(self, future):
        deadline = time.monotonic() + self.timeout
        while not future.done() and time.monotonic() < deadline:
            self.spin(min(0.02, max(0.0, deadline - time.monotonic())))
        if not future.done():
            self._connected = False
            raise TimeoutError(
                "ROS operation timed out; reconnect before sending commands"
            )
        return future.result()

    def connect(self) -> JointObservation:
        """Discover controller and require a new sample; never replay old goals."""
        self._connected = False
        self._sample = None
        if not self.client.wait_for_server(timeout_sec=self.timeout):
            raise ConnectionError("Trajectory action server is unavailable")
        if self._goal is not None or self._pending_goal is not None:
            self.cancel()
        deadline = time.monotonic() + self.timeout
        while self._sample is None and time.monotonic() < deadline:
            self.spin()
        if self._sample is None:
            raise TimeoutError("No new JointState sample after connection")
        self._connected = True
        return self.observation()

    def observation(self) -> JointObservation:
        """Return fresh measured q/v or fail instead of serving cached state."""
        if not self._connected or self._sample is None:
            raise ConnectionError("Connect and receive a fresh observation first")
        sample = self._sample
        source_age = self.node.get_clock().now().nanoseconds * 1e-9 - sample.stamp
        if (
            time.monotonic() - sample.received > self.max_age
            or source_age > self.max_age
            or source_age < -self.max_age
        ):
            self._connected = False
            raise TimeoutError("Stale or incompatible-clock joint observation")
        return sample

    def send(self, points: tuple[TrajectoryPoint, ...]) -> None:
        """Send one goal; reject replacement until the previous goal is settled."""
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint

        self.observation()
        if self._goal is not None or self._pending_goal is not None:
            raise RuntimeError(
                "Finish or cancel the active trajectory before replacing it"
            )
        if not self.client.server_is_ready():
            self._connected = False
            raise ConnectionError("Trajectory server disconnected")
        if not points:
            raise ValueError("A trajectory needs at least one point")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(self.names)
        previous = 0.0
        for point in points:
            if len(point.positions) != len(self.names) or (
                point.velocities and len(point.velocities) != len(self.names)
            ):
                raise ValueError(
                    "Trajectory dimensions differ from declared joint names"
                )
            if not all(
                math.isfinite(x)
                for x in (*point.positions, *point.velocities, point.time_from_start)
            ):
                raise ValueError("Trajectory values must be finite")
            if point.time_from_start <= previous:
                raise ValueError(
                    "Trajectory times must be positive and strictly increasing"
                )
            previous = point.time_from_start
            native = JointTrajectoryPoint()
            native.positions = list(point.positions)
            native.velocities = list(point.velocities)
            nanoseconds = round(point.time_from_start * 1e9)
            native.time_from_start.sec, native.time_from_start.nanosec = divmod(
                nanoseconds, 1_000_000_000
            )
            goal.trajectory.points.append(native)
        self._pending_goal = self.client.send_goal_async(goal)
        self._settle_pending()

    def _settle_pending(self) -> None:
        # Keep the request on timeout: it may be accepted remotely after our wait.
        # Reconnection must settle/cancel it before another goal can be sent.
        handle = self._wait(self._pending_goal)
        self._pending_goal = None
        if not handle.accepted:
            raise RuntimeError("Trajectory rejected by controller")
        self._goal = handle
        self._result = handle.get_result_async()

    def finish(self) -> None:
        """Wait for the active action result and require controller success."""
        from action_msgs.msg import GoalStatus
        from control_msgs.action import FollowJointTrajectory

        if self._result is None:
            raise RuntimeError("No active trajectory")
        result = self._wait(self._result)
        self._goal = None
        self._result = None
        if (
            result.status != GoalStatus.STATUS_SUCCEEDED
            or result.result.error_code != FollowJointTrajectory.Result.SUCCESSFUL
        ):
            raise RuntimeError(
                f"Trajectory failed: {result.status}, {result.result.error_code}: {result.result.error_string}"
            )

    def cancel(self) -> None:
        """Request cancellation, and wait for its acknowledgement and terminal state."""
        if self._pending_goal is not None:
            self._settle_pending()
        if self._goal is None:
            return
        if not self._result.done():
            response = self._wait(self._goal.cancel_goal_async())
            if not response.goals_canceling and not self._result.done():
                raise RuntimeError(
                    "Controller did not acknowledge trajectory cancellation"
                )
        self._wait(self._result)
        self._goal = None
        self._result = None

    def reset(self) -> JointObservation:
        """Cancel and reacquire state; reset does not move or home the robot."""
        self.cancel()
        return self.connect()

    def disconnect(self) -> None:
        """Invalidate observations; retain the goal for cancellation on reconnect."""
        self._connected = False
        self._sample = None

    def close(self) -> None:
        """Release local resources even when remote cancellation fails."""
        try:
            self.cancel()
        finally:
            self.executor.shutdown()
            self.node.destroy_node()
            self.context.shutdown()
            self._connected = False
