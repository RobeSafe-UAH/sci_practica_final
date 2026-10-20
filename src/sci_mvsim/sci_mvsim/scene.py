"""Visual scenery for the SCI circuit; collision geometry lives in the ROS map.

SPDX-License-Identifier: Apache-2.0
"""

import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw
import yaml


def _tag(parent, name, value):
    ET.SubElement(parent, name).text = str(value)


def _offset(center, index, distance):
    x, y = center.points[index]
    yaw = center.headings[index]
    return x + distance * math.sin(yaw), y - distance * math.cos(yaw)


def _surface(config, map_yaml, paths, output):
    spec = yaml.safe_load(Path(map_yaml).read_text(encoding='utf-8'))
    with Image.open(Path(map_yaml).parent / spec['image']) as source:
        road_mask = source.convert('L').point([0 if value < 128 else 255 for value in range(256)])
    resolution, origin = spec['resolution'], spec['origin']
    noise = Image.frombytes('L', road_mask.size,
                            random.Random(2026).randbytes(road_mask.width * road_mask.height))

    def material(color, scale):
        return Image.merge('RGB', tuple(noise.point([
            max(0, min(255, channel + (value - 128) // scale)) for value in range(256)
        ]) for channel in color))

    floor = material((62, 91, 62), 12)
    floor.paste(material((54, 59, 65), 24), (0, 0), road_mask)
    draw = ImageDraw.Draw(floor)

    def pixel(point):
        return ((point[0] - origin[0]) / resolution - 0.5,
                floor.height - 0.5 - (point[1] - origin[1]) / resolution)

    center, half = paths['centerline'], config['track']['width'] / 2
    wall = config['track']['wall_thickness']
    edge_inset = min(0.18, config['track']['width'] * 0.04)
    for side in (-1, 1):
        edge = [pixel(_offset(center, i, side * (half - edge_inset)))
                for i in range(len(center.points))]
        draw.line(edge + edge[:1], fill='#e8edf2', width=max(1, round(0.08 / resolution)))
        for i in range(len(center.points)):
            following = (i + 1) % len(center.points)
            color = '#e8edf2' if int(center.cumulative[i] / 0.8) % 2 == 0 else '#ba3e3b'
            draw.line([pixel(_offset(center, j, side * (half + wall / 2)))
                       for j in (i, following)], fill=color, width=max(1, round(wall / resolution)))
    distance = 0.0
    while distance < center.length:
        dash = [pixel(center.pose_at(min(distance + delta, center.length))[:2])
                for delta in (0.0, 0.2, 0.4, 0.6)]
        draw.line(dash, fill='#e6cf87', width=max(1, round(0.10 / resolution)))
        distance += 1.5

    for name in ('right_lane', 'opposing_lane'):
        lane = paths[name]
        for distance in range(8, int(lane.length), 24):
            x, y, yaw = lane.pose_at(distance)
            c, s = math.cos(yaw), math.sin(yaw)
            arrow = [(-0.6, -0.10), (0.05, -0.10), (0.05, -0.30),
                     (0.65, 0.0), (0.05, 0.30), (0.05, 0.10), (-0.6, 0.10)]
            draw.polygon([pixel((x + c * u - s * v, y + s * u + c * v))
                          for u, v in arrow], fill='#dce3e7')

    # A painted start line on the first straight, ahead of the initial ego pose.
    x, y, yaw = center.pose_at(config['robot']['spawn_s'] + 2.0)
    c, s = math.cos(yaw), math.sin(yaw)
    columns = max(2, round(2 * half / 0.2))
    square = 2 * half / columns
    for row in range(2):
        for column in range(columns):
            u, v = row * square, -half + column * square
            corners = ((u, v), (u + square, v),
                       (u + square, v + square), (u, v + square))
            draw.polygon([pixel((x + c * a - s * b, y + s * a + c * b))
                          for a, b in corners],
                         fill='#edf0ed' if (row + column) % 2 else '#343940')
    # MVSim maps the first texture row to y_min; ROS maps put y_max there.
    floor.transpose(Image.Transpose.FLIP_TOP_BOTTOM).save(output / 'circuit_surface.png')
    return origin, (floor.width * resolution, floor.height * resolution)


def _barriers(config, center, output):
    """One material-grouped mesh for both boundaries, rather than many blocks."""
    material = output / 'curbs.mtl'
    material.write_text(
        'newmtl white\nKd 0.83 0.87 0.88\nKs 0.08 0.08 0.08\nNs 16\n'
        'newmtl red\nKd 0.73 0.18 0.17\nKs 0.08 0.08 0.08\nNs 16\n', encoding='ascii')
    half, wall, height = config['track']['width'] / 2, config['track']['wall_thickness'], 0.25
    indices = list(range(0, len(center.points), 4))
    lines, vertex_count, normal_count = ['mtllib curbs.mtl'], 0, 0

    def face(vertices):
        nonlocal vertex_count, normal_count
        a, b, c = vertices[:3]
        ab, ac = [b[i] - a[i] for i in range(3)], [c[i] - a[i] for i in range(3)]
        normal = (ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2],
                  ab[0] * ac[1] - ab[1] * ac[0])
        norm = math.sqrt(sum(value * value for value in normal))
        lines.extend(f'v {x:.6f} {y:.6f} {z:.6f}' for x, y, z in vertices)
        lines.append('vn ' + ' '.join(f'{value / norm:.6f}' for value in normal))
        normal_count += 1
        lines.append('f ' + ' '.join(f'{vertex_count + i + 1}//{normal_count}' for i in range(4)))
        vertex_count += 4

    for side in (-1, 1):
        for k, i in enumerate(indices):
            j = indices[(k + 1) % len(indices)]
            a, b = (_offset(center, index, side * half) for index in (i, j))
            c, d = (_offset(center, index, side * (half + wall)) for index in (i, j))
            color = 'white' if int(center.cumulative[i] / 0.8) % 2 == 0 else 'red'
            lines.append(f'usemtl {color}')
            surfaces = (
                ((*a, height), (*c, height), (*d, height), (*b, height)),
                ((*a, 0), (*a, height), (*b, height), (*b, 0)),
                ((*c, 0), (*d, 0), (*d, height), (*c, height)),
            )
            for vertices in surfaces:
                face(vertices if side > 0 else vertices[::-1])
    mesh = output / 'curbs.obj'
    mesh.write_text('\n'.join(lines) + '\n', encoding='ascii')
    return mesh


def add_scene(world, config, map_yaml, paths, output):
    origin, size = _surface(config, map_yaml, paths, output)
    lights = ET.SubElement(world, 'lights')
    for name, value in (('enable_shadows', 'true'), ('shadow_map_size', 1024),
                        ('light_azimuth_deg', 45), ('light_elevation_deg', 65),
                        ('light_ambient', 0.45), ('light_diffuse', 0.75), ('light_specular', 0.25)):
        _tag(lights, name, value)
    ground = ET.SubElement(world, 'element', {'class': 'horizontal_plane'})
    for name, value in (('x_min', origin[0]), ('y_min', origin[1]),
                        ('x_max', origin[0] + size[0]), ('y_max', origin[1] + size[1]),
                        ('z', 0.002), ('texture', output / 'circuit_surface.png'),
                        ('texture_size_x', size[0]), ('texture_size_y', size[1])):
        _tag(ground, name, value)
    scenery = ET.SubElement(world, 'block', name='circuit_scenery')
    for name, value in (('static', 'true'), ('intangible', 'true'),
                        ('zmin', 0.0), ('zmax', 0.001), ('init_pose', '0 0 0')):
        _tag(scenery, name, value)
    visual = ET.SubElement(scenery, 'visual')
    _tag(visual, 'model_uri', _barriers(config, paths['centerline'], output))
