"""Closed paths and constant-width roads, independent of ROS.

SPDX-License-Identifier: Apache-2.0
Arc-length interpolation is adapted from RobeSafe-UAH/rbsf-racing,
src/base_system/rbsf_worlds/rbsf_worlds/paths.py, commit
79a61ea7c14172b5bd736b0447b58c75da19c7af (Apache-2.0).
Changes: standard-library implementation, cached distances, input validation,
and rounded rectangular roads with two opposing lanes.
"""

from bisect import bisect_right
from dataclasses import dataclass
import math


def wrap_angle(angle):
    """Return an angle in [-pi, pi), in radians."""
    return (angle + math.pi) % math.tau - math.pi


def arc_lengths(points):
    """Return cumulative waypoint distances and the length including the closing edge."""
    points = tuple(points)
    if len(points) < 3:
        raise ValueError('a closed path needs at least three points')
    cumulative = [0.0]
    total = 0.0
    for i, point in enumerate(points):
        if len(point) != 2 or not all(math.isfinite(value) for value in point):
            raise ValueError('path coordinates must be finite 2D points')
        following = points[(i + 1) % len(points)]
        segment = math.hypot(following[0] - point[0], following[1] - point[1])
        if segment <= 1e-12:
            raise ValueError('consecutive path points must differ; omit the duplicated endpoint')
        total += segment
        if i + 1 < len(points):
            cumulative.append(total)
    return cumulative, total


class ClosedPath:
    """A closed polyline traversed in waypoint order, with cached arc lengths."""

    def __init__(self, points, headings=None):
        self.points = tuple(tuple(float(value) for value in point) for point in points)
        self.cumulative, self.length = arc_lengths(self.points)
        self.headings = None if headings is None else tuple(headings)
        if self.headings is not None and (len(self.headings) != len(self.points) or
                                           not all(math.isfinite(v) for v in self.headings)):
            raise ValueError('provide one finite heading per path point')

    def pose_at(self, distance):
        """Return (x, y, heading_rad) at a travelled distance, wrapping over full laps."""
        if not math.isfinite(distance):
            raise ValueError('distance must be finite')
        distance %= self.length
        index = bisect_right(self.cumulative, distance) - 1
        x0, y0 = self.points[index]
        x1, y1 = self.points[(index + 1) % len(self.points)]
        segment = math.hypot(x1 - x0, y1 - y0)
        fraction = (distance - self.cumulative[index]) / segment
        yaw = math.atan2(y1 - y0, x1 - x0)
        if self.headings is not None:
            first = self.headings[index]
            last = self.headings[(index + 1) % len(self.points)]
            yaw = wrap_angle(first + fraction * wrap_angle(last - first))
        return x0 + fraction * (x1 - x0), y0 + fraction * (y1 - y0), yaw


@dataclass(frozen=True)
class RoundedRectangleTrack:
    """Four straights joined tangentially by four circular quarter turns.

    length and height describe the centreline's bounding rectangle; width is
    the total road width. Positive offsets point outside the rectangle.
    """

    length: float
    height: float
    corner_radius: float
    width: float
    direction: str = 'ccw'

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in
                   (self.length, self.height, self.corner_radius, self.width)):
            raise ValueError('track dimensions must be finite and positive')
        if 2 * self.corner_radius >= min(self.length, self.height):
            raise ValueError('corner_radius must leave four nonempty straights')
        if self.width / 2 >= self.corner_radius:
            raise ValueError('road width must leave a positive inner curve radius')
        if self.direction not in ('cw', 'ccw'):
            raise ValueError('direction must be cw or ccw')

    @property
    def min_radius(self):
        return self.corner_radius

    @property
    def right_offset(self):
        return (1 if self.direction == 'ccw' else -1) * self.width / 4

    def path(self, offset=0.0, samples=4096, direction=None):
        if not math.isfinite(offset) or self.corner_radius + offset <= 0:
            raise ValueError('the offset curve must have a positive radius')
        if type(samples) is not int or samples < 32:
            raise ValueError('at least 32 path samples are required')
        sign = (1 if self.direction == 'ccw' else -1) if direction is None else direction
        if sign not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        hx, hy, radius = self.length / 2, self.height / 2, self.corner_radius
        cx, cy, r = hx - radius, hy - radius, radius + offset
        perimeter = 4 * (cx + cy) + math.tau * r
        points, headings = [], []

        def straight(x0, y0, x1, y1):
            count = max(1, math.ceil(samples * math.hypot(x1 - x0, y1 - y0) / perimeter))
            yaw = math.atan2(y1 - y0, x1 - x0)
            for i in range(count):
                f = i / count
                points.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
                headings.append(yaw)

        def turn(x, y, start):
            count = max(2, math.ceil(samples * math.pi * r / 2 / perimeter))
            for i in range(count):
                angle = start + math.pi * i / (2 * count)
                points.append((x + r * math.cos(angle), y + r * math.sin(angle)))
                headings.append(wrap_angle(angle + math.pi / 2))

        # Start midway along the bottom straight, clear of the curve joins.
        straight(0.0, -hy - offset, cx, -hy - offset)
        turn(cx, -cy, -math.pi / 2)
        straight(hx + offset, -cy, hx + offset, cy)
        turn(cx, cy, 0.0)
        straight(cx, hy + offset, -cx, hy + offset)
        turn(-cx, cy, math.pi / 2)
        straight(-hx - offset, cy, -hx - offset, -cy)
        turn(-cx, -cy, math.pi)
        straight(-cx, -hy - offset, 0.0, -hy - offset)
        if sign < 0:
            points = points[:1] + points[:0:-1]
            headings = [wrap_angle(yaw + math.pi)
                        for yaw in headings[:1] + headings[:0:-1]]
        return ClosedPath(points, headings)

    def right_lane(self, samples=4096):
        return self.path(self.right_offset, samples)

    def opposing_lane(self, samples=4096):
        sign = -1 if self.direction == 'ccw' else 1
        return self.path(-self.right_offset, samples, direction=sign)
