"""ROS 2 sensor and command interface for one MVSim Ackermann vehicle.

The odometry/frame arrangement and separation of MVSim topics follow
rbsf-racing's rbsf_mvsim/mvsim_adapter.py (RBSF project). This independent
adapter accepts Twist, bridges eight simulated sonars, and enforces a command
watchdog; it does not implement navigation.
"""

from __future__ import annotations

from dataclasses import dataclass
import copy
import math
import re
import time


def sonar_distance(ranges, minimum: float, maximum: float) -> float:
    """Minimum valid return, +inf for no return, -inf for a too-close return.

    Zero/NaN are invalid scan samples. Positive readings below the measurable
    minimum, and explicit -inf readings, retain the too-close meaning instead
    of being reported as free space. Units are metres.
    """
    if not (math.isfinite(minimum) and math.isfinite(maximum) and 0 < minimum < maximum):
        raise ValueError('Sonar limits must satisfy 0 < min_range < max_range')
    result = math.inf
    for raw in ranges:
        value = float(raw)
        if value == -math.inf or (math.isfinite(value) and 0 < value < minimum):
            return -math.inf
        if math.isfinite(value) and minimum <= value <= maximum:
            result = min(result, value)
    return result


@dataclass(frozen=True)
class SensorFrame:
    frame_id: str
    x: float
    y: float
    z: float
    yaw: float


def sensor_frames(config: dict) -> tuple[str, tuple[SensorFrame, ...]]:
    """Validate sensor identifiers and build the nine sensor transforms."""
    robot = config['robot']['name']
    if not isinstance(robot, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', robot):
        raise ValueError('robot.name must be a simple ROS namespace, e.g. r0')
    lidar, sonar = config['lidar'], config['sonar']
    poses = sonar['poses']
    indices = [pose['index'] for pose in poses]
    if len(indices) != 8 or set(indices) != set(range(8)):
        raise ValueError('The sonar poses must contain indices 0 through 7 exactly once')

    def frame(label, pose, default_z):
        values = (float(pose['x']), float(pose['y']), float(pose.get('z', default_z)),
                  math.radians(float(pose.get('yaw_deg', 0.0))))
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'Non-finite pose for {label}')
        return SensorFrame(f'{robot}/{label}', *values)

    frames = [frame('laser', lidar, 0.15)]
    frames.extend(frame(f"sonar{pose['index']}", pose, sonar.get('z', 0.10))
                  for pose in sorted(poses, key=lambda pose: pose['index']))
    return robot, tuple(frames)


def ackermann_command(speed: float, yaw_rate: float, max_speed: float,
                      wheelbase: float, max_steer: float) -> tuple[float, float]:
    """Bound a finite Twist to the configured Ackermann kinematics.

    max_steer is in radians. A stationary Ackermann car cannot turn in place.
    """
    if not all(math.isfinite(value) for value in (speed, yaw_rate, max_speed,
                                                 wheelbase, max_steer)):
        raise ValueError('Commands and vehicle limits must be finite')
    if max_speed <= 0 or wheelbase <= 0 or not 0 < max_steer < math.pi / 2:
        raise ValueError('Invalid Ackermann limits')
    speed = max(-max_speed, min(max_speed, speed))
    angular_limit = abs(speed) * math.tan(max_steer) / wheelbase
    return speed, max(-angular_limit, min(angular_limit, yaw_rate))


class CommandWatchdog:
    """Command expiry uses a monotonic clock, independent of ROS/simulated time."""

    def __init__(self, max_speed: float, wheelbase: float, max_steer: float,
                 timeout: float = 1.0):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('The command timeout must be positive')
        ackermann_command(0.0, 0.0, max_speed, wheelbase, max_steer)
        self.limits = max_speed, wheelbase, max_steer
        self.timeout = timeout
        self.received_at = None
        self.latest = (0.0, 0.0)

    def accept(self, speed: float, yaw_rate: float, now: float) -> tuple[float, float]:
        """Invalid commands clear the previous command and fail closed."""
        self.latest = (0.0, 0.0)
        self.received_at = None
        if not math.isfinite(now):
            raise ValueError('A finite monotonic timestamp is required')
        self.latest = ackermann_command(speed, yaw_rate, *self.limits)
        self.received_at = now
        return self.latest

    def output(self, now: float) -> tuple[float, float]:
        if (self.received_at is None or not math.isfinite(now)
                or now < self.received_at or now - self.received_at >= self.timeout):
            return 0.0, 0.0
        return self.latest


def make_bridge_class():
    """Import ROS only when running the node; numeric helpers need no ROS."""
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    from sci_mvsim.world import load_config

    class SimulationBridge(Node):
        def __init__(self):
            super().__init__('simulation_bridge')
            self.declare_parameter('config_file', '/opt/sci/config/scenario.yaml')
            config = load_config(self.get_parameter('config_file').value)
            self.robot, self.frames = sensor_frames(config)
            self.base_frame = f'{self.robot}/base_link'
            self.odom_frame = f'{self.robot}/odom'
            self.raw_prefix = f'/_sim/{self.robot}'
            sonar = config['sonar']
            self.sonar_min = float(sonar['min_range'])
            self.sonar_max = float(sonar['max_range'])
            sonar_distance([], self.sonar_min, self.sonar_max)
            self.sonar_fov = math.radians(float(sonar['fov_degrees']))
            robot = config['robot']
            self.watchdog = CommandWatchdog(
                float(robot.get('max_speed', 1.0)), float(robot['wheelbase']),
                math.radians(float(robot.get('max_steer_deg', 24.0))), timeout=1.0)

            self.cmd_pub = self.create_publisher(Twist, f'{self.raw_prefix}/cmd_vel', 10)
            self.scan_pub = self.create_publisher(LaserScan, f'/{self.robot}/scan', 10)
            self.odom_pub = self.create_publisher(Odometry, f'/{self.robot}/odom', 10)
            self.truth_pub = self.create_publisher(
                Odometry, f'/{self.robot}/ground_truth/odom', 10)
            self.sonar_pubs = [self.create_publisher(
                Range, f'/{self.robot}/sonar{index}', 10) for index in range(8)]
            self.tf_pub = TransformBroadcaster(self)
            self.static_tf_pub = StaticTransformBroadcaster(self)
            self.publish_static_frames()

            self.create_subscription(Twist, f'/{self.robot}/cmd_vel', self.command, 10)
            self.create_subscription(
                LaserScan, f'{self.raw_prefix}/scan', self.scan, qos_profile_sensor_data)
            self.create_subscription(
                Odometry, f'{self.raw_prefix}/base_pose_ground_truth', self.odometry,
                qos_profile_sensor_data)
            for index in range(8):
                self.create_subscription(
                    LaserScan, f'{self.raw_prefix}/sonar{index}_scan',
                    lambda msg, index=index: self.sonar(msg, index), qos_profile_sensor_data)
            # A paused /clock must not suspend the command timeout.
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.command_timer = self.create_timer(
                0.05, self.command_tick, clock=self.steady_clock)
            self.get_logger().info(
                f'Simulation interface /{self.robot}; Twist command timeout: 1.0 s')

        def publish_static_frames(self):
            transforms = []
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.child_frame_id = self.odom_frame
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
            for frame in self.frames:
                transform = TransformStamped()
                transform.header.frame_id = self.base_frame
                transform.child_frame_id = frame.frame_id
                transform.transform.translation.x = frame.x
                transform.transform.translation.y = frame.y
                transform.transform.translation.z = frame.z
                transform.transform.rotation.z = math.sin(frame.yaw / 2)
                transform.transform.rotation.w = math.cos(frame.yaw / 2)
                transforms.append(transform)
            self.static_tf_pub.sendTransform(transforms)

        def publish_command(self, command):
            msg = Twist()
            msg.linear.x, msg.angular.z = command
            self.cmd_pub.publish(msg)

        def command(self, msg):
            try:
                command = self.watchdog.accept(msg.linear.x, msg.angular.z, time.monotonic())
            except ValueError:
                self.get_logger().warning('Rejected a non-finite velocity; stopping')
                command = (0.0, 0.0)
            self.publish_command(command)

        def command_tick(self):
            self.publish_command(self.watchdog.output(time.monotonic()))

        def scan(self, msg):
            output = copy.deepcopy(msg)
            output.header.frame_id = self.frames[0].frame_id
            self.scan_pub.publish(output)

        def sonar(self, msg, index):
            output = Range()
            output.header.stamp = msg.header.stamp
            output.header.frame_id = self.frames[index + 1].frame_id
            output.radiation_type = Range.ULTRASOUND
            output.field_of_view = self.sonar_fov
            output.min_range = self.sonar_min
            output.max_range = self.sonar_max
            output.range = sonar_distance(msg.ranges, self.sonar_min, self.sonar_max)
            # The minimum of several noisy rays does not retain each ray's
            # variance; zero denotes an unknown Range variance in ROS.
            output.variance = 0.0
            self.sonar_pubs[index].publish(output)

        def odometry(self, msg):
            output = copy.deepcopy(msg)
            output.header.frame_id = 'map'
            output.child_frame_id = self.base_frame
            self.truth_pub.publish(output)
            output.header.frame_id = self.odom_frame
            self.odom_pub.publish(output)
            transform = TransformStamped()
            transform.header = output.header
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = output.pose.pose.position.x
            transform.transform.translation.y = output.pose.pose.position.y
            transform.transform.translation.z = output.pose.pose.position.z
            transform.transform.rotation = output.pose.pose.orientation
            self.tf_pub.sendTransform(transform)

    return SimulationBridge


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException

    rclpy.init(args=args)
    node = None
    try:
        node = make_bridge_class()()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                node.publish_command((0.0, 0.0))
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
