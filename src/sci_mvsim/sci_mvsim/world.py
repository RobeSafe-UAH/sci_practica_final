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
import os
from pathlib import Path
import re
import shutil
import xml.etree.ElementTree as ET

from PIL import Image, ImageChops, ImageDraw
import yaml

from .geometry import ClosedPath, EllipseTrack, wrap_angle


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
    track.setdefault('name', 'sci_ellipse')
    if not isinstance(track['name'], str) or not re.fullmatch(r'[a-z][a-z0-9_]*', track['name']):
        raise ValueError('track.name must identify an installed track, without slashes')
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


def _resource_directory(name):
    source = Path(__file__).resolve().parents[1] / name
    if source.is_dir():
        return source
    # Keep offline generation/test imports independent of a running ROS installation.
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('sci_mvsim')) / name


def generate_track(config, out_dir):
    """Explicitly create an RBSF-format track asset, never called during simulation.

    The ROS map has a white closed driving corridor and occupied black infield
    and outside. The CSV uses RBSF's x/y/right-width/left-width format. Runtime
    applies an outline copy just as rbsf_mvsim does, rather than changing this map.
    """
    config = validate_config(config)
    track_config = config['track']
    output = Path(out_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
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

    # Classify at cell centres with a small subpixel rasterization error.
    scale = 4
    occupancy = Image.new('L', (columns * scale, rows * scale), 0)
    drawing = ImageDraw.Draw(occupancy)
    for offset, color in ((half_width, 255), (-half_width, 0)):
        drawing.polygon(boundary(offset, scale), fill=color)
    occupancy = occupancy.resize((columns, rows), resample=Image.Resampling.NEAREST)
    name = track_config['name']
    occupancy.save(output / f'{name}.png')
    geometry = {key: track_config[key] for key in ('semi_major', 'semi_minor', 'width', 'direction')}
    map_spec = {'image': f'{name}.png', 'resolution': resolution,
                'origin': [*origin, 0.0], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.196,
                'centerline': f'{name}_centerline.csv', 'sci_geometry': geometry}
    with (output / f'{name}.yaml').open('w', encoding='utf-8') as handle:
        yaml.safe_dump(map_spec, handle, sort_keys=False)
    with (output / f'{name}_centerline.csv').open('w', newline='', encoding='utf-8') as handle:
        handle.write('# x_m, y_m, w_tr_right_m, w_tr_left_m\n')
        writer = csv.writer(handle, lineterminator='\n')
        for point in road.path().points:
            writer.writerow([f'{point[0]:.9f}', f'{point[1]:.9f}',
                             f'{half_width:.9f}', f'{half_width:.9f}'])
    return output / f'{name}.yaml'


def outline_map(map_yaml, out_dir, thickness):
    """Create MVSim's thin-wall copy with RBSF's four-connected dilation rule."""
    map_yaml, output = Path(map_yaml), Path(out_dir)
    spec = yaml.safe_load(map_yaml.read_text(encoding='utf-8'))
    with Image.open(map_yaml.parent / spec['image']) as source:
        image = source.convert('L')
    negate, threshold = int(spec.get('negate', 0)), spec['occupied_thresh']
    occupied = image.point([255 if (value if negate else 255 - value) / 255 > threshold else 0
                            for value in range(256)])
    near_free = ImageChops.invert(occupied)
    for _ in range(max(1, math.ceil(thickness / spec['resolution']))):
        grown = near_free.copy()
        for delta in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            shifted = Image.new('L', image.size, 0)
            shifted.paste(near_free, delta)
            grown = ImageChops.lighter(grown, shifted)
        near_free = grown
    walls = ImageChops.invert(ImageChops.darker(occupied, near_free))
    stem = f'{map_yaml.stem}_outline'
    walls.save(output / f'{stem}.png')
    spec.update(image=f'{stem}.png', negate=0)
    if 'centerline' in spec:
        reference = (map_yaml.parent / spec['centerline']).resolve()
        spec['centerline'] = os.path.relpath(reference, output.resolve())
    destination = output / f'{stem}.yaml'
    destination.write_text(yaml.safe_dump(spec, sort_keys=False), encoding='utf-8')
    return destination


def _track_paths(map_yaml, direction=None):
    """Use the installed track's centreline and side widths for both legal lanes."""
    spec = yaml.safe_load(Path(map_yaml).read_text(encoding='utf-8'))
    points, right_widths, left_widths = [], [], []
    with (Path(map_yaml).parent / spec['centerline']).open(encoding='utf-8') as handle:
        for line in handle:
            if not line.strip() or line.lstrip().startswith('#'):
                continue
            x, y, right, left = map(float, line.split(','))
            if not all(math.isfinite(v) for v in (x, y, right, left)) or min(right, left) <= 0:
                raise ValueError('track reference must have finite points and positive side widths')
            points.append((x, y))
            right_widths.append(right)
            left_widths.append(left)
    original_direction = spec['sci_geometry']['direction']
    if direction is not None and direction != original_direction:
        # Keep the same starting cross-section while reversing traversal. The
        # old left side becomes the new right side, including asymmetric widths.
        points = points[:1] + points[:0:-1]
        previous_right = right_widths
        right_widths = left_widths[:1] + left_widths[:0:-1]
        left_widths = previous_right[:1] + previous_right[:0:-1]
    center = ClosedPath(points)

    def with_tangents(values):
        headings = [math.atan2(values[(i + 1) % len(values)][1] - values[i - 1][1],
                              values[(i + 1) % len(values)][0] - values[i - 1][0])
                    for i in range(len(values))]
        return ClosedPath(values, headings)

    center = with_tangents(center.points)
    right, opposing = [], []
    for i, (x, y) in enumerate(center.points):
        yaw = center.headings[i]
        nx, ny = math.sin(yaw), -math.cos(yaw)
        right.append((x + nx * right_widths[i] / 2, y + ny * right_widths[i] / 2))
        opposing.append((x - nx * left_widths[i] / 2, y - ny * left_widths[i] / 2))
    # Reverse the other lane around the same starting cross-section.
    opposing = opposing[:1] + opposing[:0:-1]
    return {'centerline': center, 'right_lane': with_tangents(right),
            'opposing_lane': with_tangents(opposing)}


def _installed_track(config):
    track = config['track']
    directory = _resource_directory('tracks') / track['name']
    path = directory / f'{track["name"]}.yaml'
    if not path.is_file():
        raise ValueError(f'installed track is missing: {path}')
    spec = yaml.safe_load(path.read_text(encoding='utf-8'))
    geometry = spec.get('sci_geometry', {})
    if (any(geometry.get(key) != track[key] for key in ('semi_major', 'semi_minor', 'width')) or
            spec['resolution'] != track['resolution']):
        raise ValueError('track dimensions are asset metadata; regenerate the track explicitly '
                         'with python3 -m sci_mvsim.world --generate-track before running')
    return path


def _track_visual(map_yaml, paths, output):
    """Keep RBSF's white/black occupancy-map appearance and add a grey lane divider."""
    spec = yaml.safe_load(Path(map_yaml).read_text(encoding='utf-8'))
    with Image.open(Path(map_yaml).parent / spec['image']) as source:
        floor = source.convert('RGB')
    resolution, origin = spec['resolution'], spec['origin']

    def pixel(point):
        return ((point[0] - origin[0]) / resolution - 0.5,
                floor.height - 0.5 - (point[1] - origin[1]) / resolution)

    floor_drawing = ImageDraw.Draw(floor)
    center = paths['centerline']
    marking_width = max(1, round(0.06 / resolution))
    s = 0.0
    while s < center.length:
        dash = [pixel(center.pose_at(min(s + distance, center.length))[:2])
                for distance in (0.0, 0.1, 0.2, 0.3, 0.4)]
        floor_drawing.line(dash, fill='#808080', width=marking_width)
        s += 0.9
    floor.save(output / 'road_texture.png')
    return origin, (floor.width * resolution, floor.height * resolution)


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
    source_track = _installed_track(config)
    output = Path(out_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    definitions = output / 'definitions'
    definitions.mkdir(exist_ok=True)
    for source in _resource_directory('definitions').glob('*.xml'):
        shutil.copyfile(source, definitions / source.name)
    shutil.copytree(_resource_directory('assets'), output / 'assets', dirs_exist_ok=True)
    track_directory = output / 'tracks' / config['track']['name']
    shutil.copytree(source_track.parent, track_directory, dirs_exist_ok=True)
    copied_track = track_directory / source_track.name
    map_yaml = outline_map(copied_track, output, config['track']['wall_thickness'])
    paths = _track_paths(copied_track, config['track']['direction'])
    origin, size = _track_visual(map_yaml, paths, output)
    right_lane = paths['right_lane']
    track = config['track']
    road = EllipseTrack(track['semi_major'], track['semi_minor'], track['width'], track['direction'])
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
    _value(grid, 'file', map_yaml)
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
    for name, path in paths.items():
        with (output / f'{name}.csv').open('w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle, lineterminator='\n')
            writer.writerow(['s_m', 'x_m', 'y_m', 'yaw_rad'])
            samples = math.ceil(path.length / 0.05)
            for i in range(samples):
                s = path.length * i / samples
                writer.writerow([f'{s:.7f}', *(f'{v:.7f}' for v in path.pose_at(s))])
    ET.indent(world)
    destination = output / 'world.xml'
    ET.ElementTree(world).write(destination, encoding='utf-8', xml_declaration=True)
    return destination


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Explicitly regenerate the installed SCI track asset')
    parser.add_argument('--config', required=True, type=Path)
    parser.add_argument('--generate-track', required=True, type=Path, metavar='OUTPUT_DIRECTORY')
    arguments = parser.parse_args()
    print(generate_track(load_config(arguments.config), arguments.generate_track))
