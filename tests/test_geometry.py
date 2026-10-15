"""Geometric invariants of the SCI road; runnable with unittest or pytest."""

import math
import unittest

from sci_mvsim.geometry import ClosedPath, EllipseTrack, transform_point, wrap_angle


class TrackGeometryTests(unittest.TestCase):
    def test_two_lanes_have_opposite_directions_and_correct_right_side(self):
        for direction in ('ccw', 'cw'):
            road = EllipseTrack(7.0, 4.0, 2.0, direction)
            for i in range(360):
                theta = math.tau * i / 360
                center = road.point_at(theta)
                right = road.point_at(theta, road.right_offset)
                heading = road.heading_at(theta)
                tangent = math.cos(heading), math.sin(heading)
                displacement = right[0] - center[0], right[1] - center[1]
                self.assertAlmostEqual(math.hypot(*displacement), 0.5, places=12)
                self.assertAlmostEqual(sum(a * b for a, b in zip(tangent, displacement)),
                                       0.0, places=12)
                self.assertLess(tangent[0] * displacement[1] - tangent[1] * displacement[0], 0)
                # Both lane centres are a quarter-width from the dividing line.
                opposite = road.point_at(theta, -road.right_offset)
                self.assertAlmostEqual(math.dist(right, opposite), 1.0, places=12)
            first = road.right_lane().pose_at(0)
            opposite = road.opposing_lane().pose_at(0)
            self.assertAlmostEqual(abs(wrap_angle(first[2] - opposite[2])), math.pi, places=3)

    def test_offsets_preserve_width_at_every_ellipse_angle(self):
        road = EllipseTrack(7.0, 4.0, 2.0)
        for i in range(360):
            theta = math.tau * i / 360
            inner = road.point_at(theta, -1.0)
            outer = road.point_at(theta, 1.0)
            self.assertAlmostEqual(math.dist(inner, outer), 2.0, places=12)
        self.assertEqual(road.point_at(0.0, 1.0), (8.0, 0.0))
        self.assertAlmostEqual(road.point_at(math.pi / 2, 1.0)[1], 5.0)

    def test_parallel_curve_length_matches_integrated_curvature(self):
        # A convex closed curve's outward offset has perimeter P + 2*pi*offset.
        road = EllipseTrack(7.0, 4.0, 2.0)
        center, lane = road.path(), road.right_lane()
        self.assertAlmostEqual(lane.length - center.length, math.pi, delta=2e-6)
        self.assertAlmostEqual(center.length - road.opposing_lane().length, math.pi, delta=2e-6)

    def test_arc_length_progression_is_constant_speed_and_wraps(self):
        lane = EllipseTrack(7.0, 4.0, 2.0).right_lane()
        speed, dt = 0.35, 0.2
        for i in range(500):
            distance = 17.0 + i * speed * dt
            a, b = lane.pose_at(distance), lane.pose_at(distance + speed * dt)
            self.assertAlmostEqual(math.dist(a[:2], b[:2]) / dt, speed, delta=5e-5)
        before, after = lane.pose_at(5.0), lane.pose_at(5.0 + 2 * lane.length)
        for actual, expected in zip(after, before):
            self.assertAlmostEqual(actual, expected, places=11)

    def test_path_interpolation_and_negative_distance(self):
        square = ClosedPath([(0, 0), (2, 0), (2, 2), (0, 2)])
        self.assertEqual(square.length, 8.0)
        self.assertEqual(square.cumulative, [0, 2, 4, 6])
        self.assertEqual(square.pose_at(3), (2, 1, math.pi / 2))
        self.assertEqual(square.pose_at(-1), (0, 1, -math.pi / 2))
        with self.assertRaises(ValueError):
            ClosedPath([(0, 0), (1, 0), (0, 0)])
        with self.assertRaises(ValueError):
            square.pose_at(float('nan'))

    def test_ellipse_rejects_folded_or_invalid_offsets(self):
        for args in ((7, 4, 0), (7, 4, 5), (4, 7, 2), (7, 4, 2, 'left')):
            with self.subTest(args=args), self.assertRaises(ValueError):
                EllipseTrack(*args)
        road = EllipseTrack(7, 4, 2)
        with self.assertRaises(ValueError):
            road.path(-road.min_radius)

    def test_curvature_and_steering_feasibility(self):
        road = EllipseTrack(7, 4, 2)
        radius = 1 / road.curvature_at(0, road.right_offset)
        self.assertAlmostEqual(radius, 16 / 7 + 0.5)
        required_steering = math.atan(0.3302 / radius)
        self.assertLess(required_steering, math.radians(24))

    def test_sensor_ranges_start_at_each_emitter(self):
        for robot_yaw in (0.0, 0.7, -2.1):
            pose = (2.0, -1.0, robot_yaw)
            for index in range(8):
                sensor_yaw = index * math.pi / 4
                local = (0.2 * math.cos(sensor_yaw), 0.2 * math.sin(sensor_yaw))
                origin = transform_point(pose, local)
                beam = robot_yaw + sensor_yaw
                hit = origin[0] + 2 * math.cos(beam), origin[1] + 2 * math.sin(beam)
                self.assertAlmostEqual(math.dist(origin, hit), 2.0, places=12)
                # A central LiDAR cannot substitute for the displaced emitter.
                self.assertNotAlmostEqual(math.dist(pose[:2], hit), 2.0, places=4)


if __name__ == '__main__':
    unittest.main()
