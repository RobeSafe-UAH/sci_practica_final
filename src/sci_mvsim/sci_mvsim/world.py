"""Validate the SCI scenario and generate a self-contained MVSim world.

SPDX-License-Identifier: Apache-2.0
The occupancy-grid world/include structure follows RobeSafe-UAH/rbsf-racing,
src/base_system/rbsf_mvsim/rbsf_mvsim/world.py, commit
79a61ea7c14172b5bd736b0447b58c75da19c7af (Apache-2.0).
Changes: no ROS dependency in the generator, constant-width ellipse, opposing
lanes, floor-only road markings, independent sonar origins, and native animated
traffic blocks. Walls contain only thin occupied bands, as in RBSF's outline_map.
"""

from copy import deepcopy
import csv
import math
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml

from .geometry import EllipseTrack, wrap_angle


def _number(section, key, *, positive=False, nonnegative=False, default=None):
    value = section.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} must be a number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{key} must be finite')
    if positive and value <= 0.0:
        raise ValueError(f'{key} must be positive')
    if nonnegative and value < 0.0:
        raise ValueError(f'{key} must be nonnegative')
    section[key] = value
    return value


def _integer(section, key, minimum):
    value = section.get(key)
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f'{key} must be an integer >= {minimum}')
    return value


def validate_config(config):
    """Return a normalized copy or raise ValueError before any output is written."""
    if not isinstance(config, dict):
        raise ValueError('scenario must be a YAML mapping')
    config = deepcopy(config)
    for name in ('track', 'robot', 'traffic', 'lidar', 'sonar'):
        if not isinstance(config.get(name), dict):
            raise ValueError(f'{name} must be a mapping')
    track, robot, traffic = (config[key] for key in ('track', 'robot', 'traffic'))
    for key in ('semi_major', 'semi_minor', 'width', 'wall_thickness', 'resolution'):
        _number(track, key, positive=True)
    road = EllipseTrack(track['semi_major'], track['semi_minor'],
                        track['width'], track.get('direction'))
    if track['width'] / 2.0 + track['wall_thickness'] >= road.min_radius:
        raise ValueError('inner wall offset folds; reduce width or wall_thickness')
    if track['wall_thickness'] < 2.0 * track['resolution']:
        raise ValueError('wall_thickness must cover at least two occupancy-grid cells')
    if track['resolution'] > 0.1:
        raise ValueError('resolution must be <= 0.1 m for the small car footprint')

    if not isinstance(robot.get('name'), str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*',
                                                               robot['name']):
        raise ValueError('robot.name must be a simple ROS name, without slashes')
    _number(robot, 'wheelbase', positive=True)
    _number(robot, 'spawn_s', nonnegative=True)
    _number(robot, 'max_speed', positive=True, default=1.0)
    steering = _number(robot, 'max_steer_deg', positive=True, default=24.0)
    if steering >= 90.0:
        raise ValueError('max_steer_deg must be less than 90 degrees')
    lane_radius = road.min_radius + road.right_offset
    if robot['wheelbase'] / math.tan(math.radians(steering)) >= lane_radius:
        raise ValueError('the right lane requires more steering than the robot permits')
    # The greatest distance from the rear-axle origin to the fixed chassis polygon.
    if track['width'] / 4.0 <= math.hypot(0.45, 0.15) + track['resolution'] / 2.0:
        raise ValueError('the configured lanes are too narrow for the F1TENTH footprint')

    count = _integer(traffic, 'count', 0)
    _number(traffic, 'speed', positive=True)
    if traffic['speed'] >= robot['max_speed']:
        raise ValueError('traffic.speed must be below robot.max_speed')
    distances = traffic.get('start_distances')
    if not isinstance(distances, list) or len(distances) != count:
        raise ValueError('start_distances must contain one distance per traffic car')
    lane = road.right_lane()
    if robot['spawn_s'] >= lane.length:
        raise ValueError('spawn_s must be smaller than the right-lane lap length')
    for i, value in enumerate(distances):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError('start_distances must be numbers')
        if not math.isfinite(value) or not 0.0 < value < lane.length:
            raise ValueError('start_distances must be positive and less than a lap')
        distances[i] = float(value)
    offsets = sorted([0.0, *distances])
    gaps = [b - a for a, b in zip(offsets, offsets[1:] + [lane.length])]
    if any(gap < 1.0 for gap in gaps):
        raise ValueError('spawn distances must leave at least 1 m between vehicles')

    for name in ('lidar', 'sonar'):
        sensor = config[name]
        _number(sensor, 'fov_degrees', positive=True)
        if sensor['fov_degrees'] > 360.0:
            raise ValueError(f'{name}.fov_degrees must be <= 360')
        _integer(sensor, 'nrays', 2)
        _number(sensor, 'period', positive=True)
        _number(sensor, 'max_range', positive=True)
    lidar, sonar = config['lidar'], config['sonar']
    for key in ('x', 'y', 'z'):
        _number(lidar, key)
    _number(sonar, 'z', nonnegative=True, default=0.10)
    _number(sonar, 'noise_std', nonnegative=True)
    _number(sonar, 'min_range', positive=True)
    if sonar['min_range'] >= sonar['max_range']:
        raise ValueError('sonar.min_range must be smaller than max_range')
    poses = sonar.get('poses')
    if not isinstance(poses, list) or len(poses) != 8 or not all(isinstance(p, dict) for p in poses):
        raise ValueError('sonar.poses must contain exactly eight mappings')
    indices = []
    for pose in poses:
        indices.append(_integer(pose, 'index', 0))
        for key in ('x', 'y', 'yaw_deg'):
            _number(pose, key)
    if sorted(indices) != list(range(8)):
        raise ValueError('sonar indices must be unique and exactly 0..7')
    sonar['poses'].sort(key=lambda pose: pose['index'])
    return config


def load_config(path):
    """Read YAML safely and validate dimensions, motion limits, and all sensor poses."""
    with Path(path).open(encoding='utf-8') as handle:
        return validate_config(yaml.safe_load(handle))


def _definitions_directory():
    source = Path(__file__).resolve().parents[1] / 'definitions'
    if source.is_dir():
        return source
    # Keep offline generation/test imports independent of a running ROS installation.
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('sci_mvsim')) / 'definitions'


def _raster_maps(track_config, output):
    """Draw thin collision walls separately from decorative road/centre-line texture."""
    road = EllipseTrack(track_config['semi_major'], track_config['semi_minor'],
                        track_config['width'], track_config['direction'])
    resolution = track_config['resolution']
    half_width, thickness = road.width / 2.0, track_config['wall_thickness']
    columns = math.ceil(2.0 * (road.semi_major + half_width + thickness + 1.0) / resolution)
    rows = math.ceil(2.0 * (road.semi_minor + half_width + thickness + 1.0) / resolution)
    if columns * rows > 16_000_000:
        raise ValueError('occupancy grid would exceed 16 million cells')
    origin = (-columns * resolution / 2.0, -rows * resolution / 2.0)

    def pixel(point, scale=1):
        return ((point[0] - origin[0]) / resolution * scale - 0.5,
                rows * scale - 0.5 - (point[1] - origin[1]) / resolution * scale)

    def boundary(offset, scale=1):
        return [pixel(road.point_at(math.tau * i / 2048, offset), scale)
                for i in range(2048)]

    # Only the two wall bands are occupied. Solid occupied infield/outside maps
    # have a known MVSim 2D-raytracing issue, documented by RBSF's world.py.
    # Supersample polygon filling, then classify at cell centres. Direct integer
    # polygon filling can erase the middle of a 2.4-cell wall at diagonal edges.
    scale = 4
    occupancy = Image.new('L', (columns * scale, rows * scale), 255)
    drawing = ImageDraw.Draw(occupancy)
    for offset, color in ((half_width + thickness, 0), (half_width, 255),
                          (-half_width, 0), (-half_width - thickness, 255)):
        drawing.polygon(boundary(offset, scale), fill=color)
    occupancy = occupancy.resize((columns, rows), resample=Image.Resampling.NEAREST)
    occupancy.save(output / 'walls.png')
    map_spec = {'image': 'walls.png', 'resolution': resolution,
                'origin': [*origin, 0.0], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.196}
    with (output / 'walls.yaml').open('w', encoding='utf-8') as handle:
        yaml.safe_dump(map_spec, handle, sort_keys=False)

    floor = Image.new('RGB', (columns, rows), '#c9d4b2')
    floor_drawing = ImageDraw.Draw(floor)
    # Repaint the occupied bands on the visual floor too: the texture sits
    # slightly above the grid's flat white map, which would otherwise hide it.
    for offset, color in ((half_width + thickness, '#20252b'),
                          (half_width, '#555b63'), (-half_width, '#20252b'),
                          (-half_width - thickness, '#c9d4b2')):
        floor_drawing.polygon(boundary(offset), fill=color)
    edge_width = max(1, round(0.06 / resolution))
    for offset in (half_width - 0.06, -half_width + 0.06):
        edge = boundary(offset)
        floor_drawing.line([*edge, edge[0]], fill='#f5f5f5', width=edge_width)
    # These pixels are a ground texture, never part of the occupancy map.
    center = road.path()
    marking_width = max(1, round(0.06 / resolution))
    s = 0.0
    while s < center.length:
        dash = [pixel(center.pose_at(min(s + distance, center.length))[:2])
                for distance in (0.0, 0.1, 0.2, 0.3, 0.4)]
        floor_drawing.line(dash, fill='#ffe579', width=marking_width)
        s += 0.9
    floor.save(output / 'road_texture.png')
    return origin, (columns * resolution, rows * resolution)


def _value(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def _vehicle_include_attributes(config):
    attributes = {key: str(config['robot'][key]) for key in ('wheelbase', 'max_steer_deg')}
    for name in ('lidar', 'sonar'):
        for key, value in config[name].items():
            if key != 'poses':
                attributes[f'{name}_{key}'] = str(value)
    for pose in config['sonar']['poses']:
        for key in ('x', 'y', 'yaw_deg'):
            attributes[f'sonar{pose["index"]}_{key}'] = str(pose[key])
    return attributes


def _traffic_animation(block, lane, initial_s, speed):
    """Use equally spaced arc-length frames and native MVSim automatic looping."""
    animation = ET.SubElement(block, 'animation', type='keyframes')
    period = lane.length / speed
    frames = math.ceil(period / 0.2)  # <= 7 cm per keyframe at the default 0.35 m/s.
    first = lane.pose_at(initial_s)
    previous_yaw = first[2]
    unwrapped_yaw = first[2]
    for i in range(frames + 1):
        time = period * i / frames
        x, y, yaw = first if i == frames else lane.pose_at(initial_s + lane.length * i / frames)
        if i:
            unwrapped_yaw += wrap_angle(yaw - previous_yaw)
        previous_yaw = yaw
        _value(animation, 'time_pose', f'{time:.8f} {x:.7f} {y:.7f} '
               f'{math.degrees(unwrapped_yaw):.7f}')


def build_world(config, out_dir):
    """Write world, map, texture, definitions, and reference CSVs; return world.xml Path.

    All referenced files are generated/copied below ``out_dir``. Traffic runs
    entirely in MVSim: Python generates keyframes once and is not a traffic loop.
    2D sensing and collisions use the thin occupied wall bands. Road markings
    are rendered by a separate ground texture and never alter this wall mask.
    """
    config = validate_config(config)
    output = Path(out_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    definitions = output / 'definitions'
    definitions.mkdir(exist_ok=True)
    for source in _definitions_directory().glob('*.xml'):
        shutil.copyfile(source, definitions / source.name)
    origin, size = _raster_maps(config['track'], output)
    track = config['track']
    road = EllipseTrack(track['semi_major'], track['semi_minor'], track['width'], track['direction'])
    right_lane = road.right_lane()
    world = ET.Element('mvsim_world', version='1.0')
    world.append(ET.Comment(' SCI: one lane per direction; slow traffic follows ego lane. '))
    _value(world, 'simul_timestep', 0.005)
    gui = ET.SubElement(world, 'gui')
    _value(gui, 'ortho', 'true')
    # Leave the left side of the viewport to MVSim's control/status panels.
    # A full lap and both lanes stay visible at the default desktop size.
    _value(gui, 'cam_distance', max(28, 3.5 * (road.semi_major + road.width / 2)))
    _value(gui, 'cam_point_to', f'{-0.375 * (road.semi_major + road.width / 2):.3f} 0 0')
    _value(gui, 'cam_elevation', 90)
    _value(gui, 'cam_azimuth', -90)
    _value(gui, 'show_forces', 'false')
    ground = ET.SubElement(world, 'element', {'class': 'horizontal_plane'})
    for tag, value in (('x_min', origin[0]), ('y_min', origin[1]),
                       ('x_max', origin[0] + size[0]), ('y_max', origin[1] + size[1]),
                       # The grid renders at z=0. A uniform 2 mm lift makes the
                       # road visible while leaving all horizontal geometry intact.
                       ('z', 0.002), ('texture', output / 'road_texture.png'),
                       ('texture_size_x', size[0]), ('texture_size_y', size[1])):
        _value(ground, tag, value)
    grid = ET.SubElement(world, 'element', {'class': 'occupancy_grid'})
    _value(grid, 'file', output / 'walls.yaml')
    ET.SubElement(world, 'include', file=str(definitions / 'ackermann.vehicle.xml'),
                  **_vehicle_include_attributes(config))
    ET.SubElement(world, 'include', file=str(definitions / 'traffic.block.xml'))
    ego = ET.SubElement(world, 'vehicle', name=config['robot']['name'], **{'class': 'sci_ego'})
    x, y, yaw = right_lane.pose_at(config['robot']['spawn_s'])
    _value(ego, 'init_pose', f'{x:.7f} {y:.7f} {math.degrees(yaw):.7f}')
    for index, distance in enumerate(config['traffic']['start_distances']):
        block = ET.SubElement(world, 'block', name=f'slow_car_{index}', **{'class': 'sci_traffic'})
        # Keyframes are world poses relative to init_pose, hence identity here.
        _value(block, 'init_pose', '0 0 0')
        _traffic_animation(block, right_lane, config['robot']['spawn_s'] + distance,
                           config['traffic']['speed'])
    for name, path in (('centerline', road.path()), ('right_lane', right_lane),
                       ('opposing_lane', road.opposing_lane())):
        with (output / f'{name}.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(['s_m', 'x_m', 'y_m', 'yaw_rad'])
            samples = math.ceil(path.length / 0.05)
            for i in range(samples):
                s = path.length * i / samples
                writer.writerow([f'{s:.7f}', *(f'{v:.7f}' for v in path.pose_at(s))])
    ET.indent(world)
    destination = output / 'world.xml'
    ET.ElementTree(world).write(destination, encoding='utf-8', xml_declaration=True)
    return destination
