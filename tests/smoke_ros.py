#!/usr/bin/env python3
"""ROS 2 acceptance checks; run beside the simulator in an isolated ROS domain.

Default operation only observes messages. --exercise-command briefly commands
the simulated ego. --traffic performs read-only MVSim get_pose requests.
No pytest dependency is required.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import time


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def require(condition, explanation):
    if not condition:
        raise AssertionError(explanation)


def finite_pose(message):
    position, quaternion = message.pose.pose.position, message.pose.pose.orientation
    values = (position.x, position.y, position.z, quaternion.x, quaternion.y,
              quaternion.z, quaternion.w, message.twist.twist.linear.x,
              message.twist.twist.angular.z)
    return all(math.isfinite(value) for value in values)


def project_to_path(path, x, y):
    """Return nearest polyline arc length and lateral distance, in metres."""
    nearest = (math.inf, 0.0)
    for index, (x0, y0) in enumerate(path.points):
        x1, y1 = path.points[(index + 1) % len(path.points)]
        dx, dy = x1 - x0, y1 - y0
        fraction = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / (dx * dx + dy * dy)))
        distance = math.hypot(x - x0 - fraction * dx, y - y0 - fraction * dy)
        if distance < nearest[0]:
            nearest = distance, path.cumulative[index] + fraction * math.hypot(dx, dy)
    return nearest[1] % path.length, nearest[0]


def traffic_snapshot_worker(names):
    """This subprocess has loader paths set before importing MVSim's extension."""
    from mvsim_comms import pymvsim_comms
    from mvsim_msgs.SrvGetPose_pb2 import SrvGetPose
    from mvsim_msgs.SrvGetPoseAnswer_pb2 import SrvGetPoseAnswer

    client = pymvsim_comms.mvsim.Client()
    client.setName(f'sci_smoke_{os.getpid()}')
    client.connect()
    poses = {}
    try:
        for name in names:
            request = SrvGetPose(objectId=name)
            answer = SrvGetPoseAnswer()
            answer.ParseFromString(client.callService('get_pose', request.SerializeToString()))
            require(answer.success and answer.HasField('pose'),
                    f'MVSim get_pose({name}) failed: {answer.errorMessage}')
            poses[name] = [answer.pose.x, answer.pose.y, answer.pose.yaw]
        print('SCI_TRAFFIC_JSON=' + json.dumps({'time': time.monotonic(), 'poses': poses}))
    finally:
        client.shutdown()


def traffic_snapshot(names, timeout):
    environment = os.environ.copy()
    prefix = Path('/opt/ros') / environment.get('ROS_DISTRO', 'jazzy')
    environment['PYTHONPATH'] = str(prefix / 'lib/python3/dist-packages') + os.pathsep + environment.get('PYTHONPATH', '')
    multiarch = sysconfig.get_config_var('MULTIARCH')
    require(bool(multiarch), 'Python did not report its native library architecture')
    environment['LD_LIBRARY_PATH'] = str(prefix / 'lib' / multiarch) + os.pathsep + environment.get('LD_LIBRARY_PATH', '')
    result = subprocess.run(
        [sys.executable, str(Path(__file__).resolve()), '--_traffic-snapshot', *names],
        env=environment, capture_output=True, text=True, timeout=timeout, check=True)
    for line in result.stdout.splitlines():
        if line.startswith('SCI_TRAFFIC_JSON='):
            return json.loads(line.split('=', 1)[1])
    raise AssertionError(f'No pose result from MVSim: {result.stdout} {result.stderr}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=25.0,
                        help='Seconds to wait for ROS topics/TF; default: 25')
    parser.add_argument('--config', '--config-file', dest='config',
                        default='/opt/sci/config/scenario.yaml')
    parser.add_argument('--exercise-command', action='store_true',
                        help='Drive the isolated simulated ego briefly and verify its watchdog')
    parser.add_argument('--traffic', action='store_true',
                        help='Inspect movement of both animated cars via read-only MVSim get_pose')
    parser.add_argument('--_traffic-snapshot', nargs='+', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if args._traffic_snapshot:
        traffic_snapshot_worker(args._traffic_snapshot)
        return
    require(math.isfinite(args.timeout) and args.timeout > 0, '--timeout must be positive')

    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, Range
    from tf2_msgs.msg import TFMessage
    import yaml

    config = yaml.safe_load(Path(args.config).read_text())
    robot = config['robot']['name']
    prefix = f'/{robot}'
    lidar, sonar = config['lidar'], config['sonar']
    expected_frames = {f'{robot}/odom': 'map', f'{robot}/base_link': f'{robot}/odom',
                       f'{robot}/laser': f'{robot}/base_link',
                       **{f'{robot}/sonar{index}': f'{robot}/base_link' for index in range(8)}}
    latest, stamps, transforms, issues = {}, {}, {}, set()
    odom_history, raw_commands = [], []
    rclpy.init()
    node = rclpy.create_node('sci_acceptance')
    command_publisher = None

    def validate_message(label, msg):
        if label == 'odom':
            require(msg.header.frame_id == f'{robot}/odom', 'Odometry parent frame differs')
            require(msg.child_frame_id == f'{robot}/base_link', 'Odometry child frame differs')
            require(finite_pose(msg), 'Odometry contains non-finite pose/velocity')
        elif label == 'scan':
            require(msg.header.frame_id == f'{robot}/laser', 'LiDAR frame differs')
            require(len(msg.ranges) == lidar['nrays'], f"LiDAR rays: {len(msg.ranges)} != {lidar['nrays']}")
            require(abs(msg.angle_max - msg.angle_min - math.radians(lidar['fov_degrees'])) < 0.002,
                    'LiDAR FOV differs from configuration')
            require(abs(msg.range_max - lidar['max_range']) < 1e-4, 'LiDAR max_range differs')
            require(all(not math.isnan(value) for value in msg.ranges), 'LiDAR contains NaN')
            require(all(not math.isfinite(value) or msg.range_min <= value <= msg.range_max
                        for value in msg.ranges), 'LiDAR contains an out-of-range finite value')
        else:
            index = int(label[5:])
            require(msg.header.frame_id == f'{robot}/sonar{index}', f'{label} frame differs')
            require(msg.radiation_type == Range.ULTRASOUND, f'{label} radiation type differs')
            require(abs(msg.field_of_view - math.radians(sonar['fov_degrees'])) < 1e-5,
                    f'{label} FOV differs')
            require(abs(msg.min_range - sonar['min_range']) < 1e-5
                    and abs(msg.max_range - sonar['max_range']) < 1e-5,
                    f'{label} range limits differ')
            require(not math.isnan(msg.range), f'{label} contains NaN')
            require(not math.isfinite(msg.range) or msg.min_range <= msg.range <= msg.max_range,
                    f'{label} finite reading is outside its limits')

    def receive(label, msg):
        latest[label] = msg
        stamp = stamp_seconds(msg.header.stamp)
        first, previous = stamps.get(label, (stamp, stamp))
        stamps[label] = first, stamp
        if stamp < previous - 1e-6:
            issues.add(f'{label} timestamps went backwards')
        if not 0 <= msg.header.stamp.nanosec < 1_000_000_000:
            issues.add(f'{label} timestamp is malformed')
        try:
            validate_message(label, msg)
        except AssertionError as error:
            issues.add(str(error))
        if label == 'odom':
            p = msg.pose.pose.position
            odom_history.append((time.monotonic(), p.x, p.y, msg.twist.twist.linear.x))

    def receive_tf(msg):
        for transform in msg.transforms:
            if transform.child_frame_id in expected_frames:
                transforms[transform.child_frame_id] = transform

    def receive_raw_command(msg):
        raw_commands.append((time.monotonic(), msg.linear.x, msg.angular.z))

    node.create_subscription(LaserScan, f'{prefix}/scan',
                             lambda msg: receive('scan', msg), qos_profile_sensor_data)
    node.create_subscription(Odometry, f'{prefix}/odom',
                             lambda msg: receive('odom', msg), qos_profile_sensor_data)
    for index in range(8):
        node.create_subscription(Range, f'{prefix}/sonar{index}',
                                 lambda msg, index=index: receive(f'sonar{index}', msg),
                                 qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf', receive_tf, qos_profile_sensor_data)
    node.create_subscription(TFMessage, '/tf_static', receive_tf, QoSProfile(
        depth=100, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL))
    if args.exercise_command:
        command_publisher = node.create_publisher(Twist, f'{prefix}/cmd_vel', 10)
        node.create_subscription(Twist, f'/_sim/{robot}/cmd_vel',
                                 receive_raw_command, qos_profile_sensor_data)

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=min(0.05, max(0.0, end - time.monotonic())))

    try:
        end = time.monotonic() + args.timeout
        missing = []
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            missing = [label for label in ('scan', 'odom', *(f'sonar{i}' for i in range(8)))
                       if label not in stamps or stamps[label][1] <= max(0, stamps[label][0])]
            missing.extend(f'TF {child}' for child in expected_frames if child not in transforms)
            if not missing:
                break
        require(not missing, 'Timed out waiting for: ' + ', '.join(missing))
        require(not issues, '; '.join(sorted(issues)))
        for child, parent in expected_frames.items():
            transform = transforms[child]
            require(transform.header.frame_id == parent, f'TF {child} has wrong parent')
            t, q = transform.transform.translation, transform.transform.rotation
            require(all(math.isfinite(value) for value in (t.x, t.y, t.z, q.x, q.y, q.z, q.w)),
                    f'TF {child} contains a non-finite value')
            require(abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) < 1e-3,
                    f'TF {child} quaternion is not normalized')
        sensor_poses = [(f'{robot}/laser', lidar, lidar.get('z', 0.15))]
        sensor_poses.extend((f"{robot}/sonar{pose['index']}", pose, sonar.get('z', 0.10))
                            for pose in sonar['poses'])
        for child, pose, default_z in sensor_poses:
            transform = transforms[child].transform
            t, q = transform.translation, transform.rotation
            yaw = math.radians(pose.get('yaw_deg', 0.0))
            require(max(abs(t.x - pose['x']), abs(t.y - pose['y']),
                        abs(t.z - pose.get('z', default_z))) < 1e-5,
                    f'TF {child} position differs from sensor configuration')
            require(abs(q.z - math.sin(yaw / 2)) < 1e-5
                    and abs(q.w - math.cos(yaw / 2)) < 1e-5,
                    f'TF {child} orientation differs from sensor configuration')
        origin = transforms[f'{robot}/odom'].transform
        require(max(abs(origin.translation.x), abs(origin.translation.y),
                    abs(origin.translation.z), abs(origin.rotation.x),
                    abs(origin.rotation.y), abs(origin.rotation.z),
                    abs(origin.rotation.w - 1.0)) < 1e-6,
                'map -> odom must be the identity for the ground-truth odometry interface')
        require(stamp_seconds(transforms[f'{robot}/base_link'].header.stamp) > 0,
                'Dynamic base transform has no source timestamp')
        print(f'PASS sensors: LiDAR {lidar["nrays"]} rays, odometry, 8 Range; advancing source stamps')
        print(f'PASS TF: {len(expected_frames)} required transforms with consistent frames')

        if args.traffic:
            from sci_mvsim.geometry import EllipseTrack, wrap_angle

            names = [f'slow_car_{index}' for index in range(config['traffic']['count'])]
            first = traffic_snapshot(names, min(10.0, args.timeout))
            spin_for(2.0)
            second = traffic_snapshot(names, min(10.0, args.timeout))
            dt = second['time'] - first['time']
            track = config['track']
            path = EllipseTrack(track['semi_major'], track['semi_minor'], track['width'],
                                track['direction']).right_lane()
            for name in names:
                p0, p1 = first['poses'][name], second['poses'][name]
                require(all(math.isfinite(value) for value in (*p0, *p1)),
                        f'{name} pose contains a non-finite value')
                s0, lateral0 = project_to_path(path, *p0[:2])
                s1, lateral1 = project_to_path(path, *p1[:2])
                travelled = (s1 - s0) % path.length
                expected = config['traffic']['speed'] * dt
                require(max(lateral0, lateral1) < 0.12, f'{name} left its right lane')
                require(abs(travelled - expected) < max(0.15, expected * 0.25),
                        f'{name} moved {travelled:.3f} m in {dt:.2f} s; expected {expected:.3f} m in driving order')
                require(abs(wrap_angle(p1[2] - path.pose_at(s1)[2])) < 0.15,
                        f'{name} heading is opposite to its driving direction')
                print(f'PASS traffic {name}: {travelled:.3f} m / {dt:.2f} s, in its right lane')

        if args.exercise_command:
            # Default acceptance is passive; this block is explicitly requested.
            start_x, start_y = odom_history[-1][1:3]
            message = Twist()
            message.linear.x = 0.2
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                command_publisher.publish(message)
                spin_for(0.05)
            stopped_sending_at = time.monotonic()
            # Send nothing for long enough to exercise the bridge's own timeout.
            spin_for(1.6)
            distance = math.hypot(odom_history[-1][1] - start_x, odom_history[-1][2] - start_y)
            require(distance > 0.08, f'The ego did not advance: displacement {distance:.3f} m')
            require(raw_commands, 'No forwarded commands observed on the internal topic')
            zero_time, speed, angular = raw_commands[-1]
            require(zero_time > stopped_sending_at + 1.0 and abs(speed) < 1e-6 and abs(angular) < 1e-6,
                    'The watchdog did not publish zero after the command stream ended')
            require(any(abs(command[1] - 0.2) < 1e-3 for command in raw_commands),
                    'The requested velocity did not reach the simulator')
            tail = [sample for sample in odom_history if sample[0] >= time.monotonic() - 0.25]
            require(len(tail) >= 2, 'Insufficient odometry to verify physical stopping')
            residual = math.hypot(tail[-1][1] - tail[0][1], tail[-1][2] - tail[0][2])
            require(residual < 0.015 and max(abs(sample[3]) for sample in tail) < 0.05,
                    f'The ego did not settle after watchdog expiry: {residual:.4f} m in the last 0.25 s')
            print(f'PASS command/watchdog: ego advanced {distance:.3f} m and stopped without further commands')
        require(not issues, '; '.join(sorted(issues)))
        print('SCI ROS acceptance passed')
    finally:
        if command_publisher is not None and rclpy.ok():
            command_publisher.publish(Twist())
            spin_for(0.05)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    try:
        main()
    except (AssertionError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f'FAIL: {error}', file=sys.stderr)
        sys.exit(1)
