"""World generation and native traffic contracts, without ROS or MVSim imports."""

from copy import deepcopy
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

from PIL import Image
import yaml

from sci_mvsim.geometry import EllipseTrack, wrap_angle
from sci_mvsim.world import build_world, load_config, validate_config


ROOT = Path(__file__).resolve().parents[1]


def parse_definition(path):
    # MVSim uses literal colon tags, not XML namespaces, in class definition files.
    text = Path(path).read_text(encoding='utf-8')
    return ET.fromstring(text.replace('vehicle:class', 'vehicle_class')
                        .replace('block:class', 'block_class'))


class WorldGenerationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_config(ROOT / 'config' / 'scenario.yaml')
        cls.temporary = tempfile.TemporaryDirectory(prefix='sci_world_test_')
        cls.directory = Path(cls.temporary.name)
        cls.world_path = build_world(cls.config, cls.directory)
        cls.world = ET.parse(cls.world_path).getroot()
        cls.lane = EllipseTrack(7, 4, 2).right_lane()

    @classmethod
    def tearDownClass(cls):
        cls.temporary.cleanup()

    def test_world_is_self_contained_with_one_ego_and_two_native_animations(self):
        self.assertEqual(self.world.tag, 'mvsim_world')
        vehicles = self.world.findall('vehicle')
        self.assertEqual([(v.get('name'), v.get('class')) for v in vehicles], [('r0', 'sci_ego')])
        traffic = self.world.findall('block')
        self.assertEqual([block.get('name') for block in traffic], ['slow_car_0', 'slow_car_1'])
        self.assertTrue(all(block.find('animation').get('type') == 'keyframes' for block in traffic))
        self.assertTrue(all(block.find('init_pose').text == '0 0 0' for block in traffic))
        for include in self.world.findall('include'):
            path = Path(include.get('file'))
            self.assertTrue(path.is_file())
            self.assertEqual(path.parent, self.directory / 'definitions')
        ego = tuple(map(float, vehicles[0].find('init_pose').text.split()))
        self.assertAlmostEqual(ego[0], 7.5, places=7)
        self.assertAlmostEqual(ego[1], 0.0, places=7)
        self.assertAlmostEqual(ego[2], 90.0, delta=0.1)

    def test_native_traffic_closes_the_lap_and_has_requested_spacing_and_speed(self):
        for block, distance in zip(self.world.findall('block'), (5.0, 17.0)):
            frames = [tuple(map(float, node.text.split()))
                      for node in block.findall('animation/time_pose')]
            self.assertGreater(len(frames), 500)
            expected = self.lane.pose_at(distance)
            self.assertAlmostEqual(frames[0][0], 0.0)
            self.assertAlmostEqual(math.dist(frames[0][1:3], expected[:2]), 0.0, delta=1e-6)
            self.assertAlmostEqual(frames[-1][0], self.lane.length / 0.35, places=6)
            self.assertEqual(frames[-1][1:3], frames[0][1:3])
            self.assertAlmostEqual(wrap_angle(math.radians(frames[-1][3] - frames[0][3])),
                                   0.0, places=8)
            for previous, following in zip(frames, frames[1:]):
                dt = following[0] - previous[0]
                self.assertGreater(dt, 0.0)
                self.assertLessEqual(dt, 0.200001)
                actual_speed = math.dist(previous[1:3], following[1:3]) / dt
                self.assertAlmostEqual(actual_speed, 0.35, delta=1e-4)
                self.assertLess(abs(following[3] - previous[3]), 3.0)

    def test_wall_grid_contains_only_walls_and_keeps_both_lanes_free(self):
        map_path = Path(self.world.find("element[@class='occupancy_grid']/file").text)
        spec = yaml.safe_load(map_path.read_text(encoding='utf-8'))
        image = Image.open(map_path.parent / spec['image'])
        self.assertEqual(spec['resolution'], 0.05)

        def occupied(x, y):
            column = math.floor((x - spec['origin'][0]) / spec['resolution'])
            row = image.height - 1 - math.floor((y - spec['origin'][1]) / spec['resolution'])
            return image.getpixel((column, row)) == 0

        road = EllipseTrack(7, 4, 2)
        for i in range(180):
            theta = i * math.tau / 180
            for offset in (-0.5, 0.0, 0.5):
                self.assertFalse(occupied(*road.point_at(theta, offset)))
            # Half a wall thickness beyond each road boundary must be occupied.
            for offset in (-1.06, 1.06):
                self.assertTrue(occupied(*road.point_at(theta, offset)))
        self.assertFalse(occupied(0, 0))
        self.assertFalse(occupied(8.5, 0))

    def test_road_markings_are_texture_and_not_collision_geometry(self):
        ground = self.world.find("element[@class='horizontal_plane']")
        self.assertIsNotNone(ground)
        # Clear the flat occupancy-map visualization without reaching a sensor
        # plane or any part of the chassis (whose lower edge is at 3 cm).
        height = float(ground.find('z').text)
        self.assertGreater(height, 0.0)
        self.assertLess(height, 0.01)
        texture = Image.open(ground.find('texture').text)
        colors = {color for _, color in texture.getcolors()}
        self.assertTrue({(255, 229, 121), (245, 245, 245), (32, 37, 43)} <= colors)
        self.assertEqual(len(self.world.findall('block')), 2)
        gui = self.world.find('gui')
        self.assertEqual(float(gui.find('cam_elevation').text), 90.0)
        self.assertEqual(float(gui.find('cam_azimuth').text), -90.0)
        self.assertGreaterEqual(float(gui.find('cam_distance').text), 22.0)

    def test_sensor_origins_and_limits_are_forwarded_to_nine_distinct_sensors(self):
        include = self.world.find('include')
        self.assertEqual(float(include.get('lidar_period')), 0.05)
        self.assertEqual(int(include.get('lidar_nrays')), 1081)
        self.assertEqual(float(include.get('sonar_period')), 0.05)
        self.assertEqual(int(include.get('sonar_nrays')), 11)
        self.assertEqual(float(include.get('sonar_min_range')), 0.04)
        self.assertEqual(float(include.get('sonar_max_range')), 5.0)
        vehicle = parse_definition(include.get('file'))
        self.assertEqual(vehicle.find('dynamics').get('class'), 'ackermann')
        self.assertEqual(vehicle.find('dynamics/controller').get('class'), 'twist_front_steer_pid')
        sensors = vehicle.findall('include')
        self.assertEqual(len(sensors), 9)
        self.assertEqual([sensor.get('sensor_name') for sensor in sensors[1:]],
                         [f'sonar{i}_scan' for i in range(8)])
        for pose in self.config['sonar']['poses']:
            prefix = f'sonar{pose["index"]}_'
            for key in ('x', 'y', 'yaw_deg'):
                self.assertEqual(float(include.get(prefix + key)), pose[key])
        sonar = parse_definition(self.directory / 'definitions' / 'sonar_scan.sensor.xml')
        self.assertEqual(sonar.find('range_std_noise').text, '${sonar_noise_std|0.005}')
        self.assertEqual(sonar.find('raytrace_3d').text, 'false')

    def test_invalid_scenario_is_rejected_before_writing_files(self):
        invalid_updates = (
            ('track', 'width', 5.0), ('track', 'resolution', 0.2),
            ('robot', 'name', '../robot'), ('traffic', 'speed', 1.1),
            ('traffic', 'start_distances', [0.2, 17.0]),
            ('sonar', 'min_range', 6.0), ('sonar', 'min_range', 0.0),
            ('sonar', 'nrays', True),
            ('lidar', 'max_range', float('nan')),
        )
        for section, key, value in invalid_updates:
            with self.subTest(section=section, key=key):
                config = deepcopy(self.config)
                config[section][key] = value
                output = self.directory / 'invalid'
                with self.assertRaises(ValueError):
                    build_world(config, output)
                self.assertFalse(output.exists())
        duplicate = deepcopy(self.config)
        duplicate['sonar']['poses'][7]['index'] = 0
        with self.assertRaises(ValueError):
            validate_config(duplicate)


if __name__ == '__main__':
    unittest.main()
