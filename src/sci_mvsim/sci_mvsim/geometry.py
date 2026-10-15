"""Closed paths and constant-width elliptical roads, independent of ROS.

SPDX-License-Identifier: Apache-2.0
Arc-length interpolation is adapted from RobeSafe-UAH/rbsf-racing,
src/base_system/rbsf_worlds/rbsf_worlds/paths.py, commit
79a61ea7c14172b5bd736b0447b58c75da19c7af (Apache-2.0).
Changes: standard-library implementation, cached distances, input validation,
and analytic normal offsets for an elliptical road with two opposing lanes.
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


def pose_at(points, distance):
    """Interpolate an uncached closed path; use ClosedPath for repeated queries."""
    return ClosedPath(points).pose_at(distance)


@dataclass(frozen=True)
class EllipseTrack:
    """A road of TOTAL width ``width`` around an ellipse with semiaxes a and b.

    The positive normal points outside. A vehicle travelling counterclockwise
    has its right lane outside; clockwise traffic has its right lane inside.
    """

    semi_major: float
    semi_minor: float
    width: float
    direction: str = 'ccw'

    def __post_init__(self):
        values = (self.semi_major, self.semi_minor, self.width)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError('ellipse semiaxes and road width must be finite and positive')
        if self.semi_major < self.semi_minor:
            raise ValueError('semi_major must be at least semi_minor')
        if self.direction not in ('ccw', 'cw'):
            raise ValueError('direction must be ccw or cw')
        if self.width / 2.0 >= self.min_radius:
            raise ValueError('the inner road offset folds: width/2 must be less than b*b/a')

    @property
    def sign(self):
        """Orientation relative to increasing ellipse parameter."""
        return 1 if self.direction == 'ccw' else -1

    @property
    def min_radius(self):
        """Minimum radius of curvature on the centre ellipse."""
        return self.semi_minor ** 2 / self.semi_major

    @property
    def right_offset(self):
        """Signed outward offset of the ego's lane centre."""
        return self.sign * self.width / 4.0

    def normal_at(self, theta):
        """Outward unit normal to the ellipse."""
        a, b = self.semi_major, self.semi_minor
        q = math.hypot(a * math.sin(theta), b * math.cos(theta))
        return b * math.cos(theta) / q, a * math.sin(theta) / q

    def point_at(self, theta, offset=0.0):
        """Point at a true normal offset, in metres, from the centre ellipse."""
        nx, ny = self.normal_at(theta)
        return (self.semi_major * math.cos(theta) + offset * nx,
                self.semi_minor * math.sin(theta) + offset * ny)

    def heading_at(self, theta, direction=None):
        """Tangent heading in the specified driving direction (radians)."""
        sign = self.sign if direction is None else direction
        if sign not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        return math.atan2(sign * self.semi_minor * math.cos(theta),
                          -sign * self.semi_major * math.sin(theta))

    def curvature_at(self, theta, offset=0.0):
        """Unsigned curvature of a regular outward-offset curve."""
        a, b = self.semi_major, self.semi_minor
        q = math.hypot(a * math.sin(theta), b * math.cos(theta))
        curvature = a * b / q ** 3
        if 1.0 + offset * curvature <= 0.0:
            raise ValueError('the offset curve is not regular')
        return curvature / (1.0 + offset * curvature)

    def path(self, offset=0.0, samples=4096, direction=None):
        """Sample a closed offset curve in driving order, without repeating the endpoint."""
        if not isinstance(samples, int) or isinstance(samples, bool) or samples < 32:
            raise ValueError('at least 32 path samples are required')
        sign = self.sign if direction is None else direction
        if sign not in (-1, 1):
            raise ValueError('direction must be +1 or -1')
        if offset <= -self.min_radius:
            raise ValueError('the offset curve is not regular')
        parameters = [sign * math.tau * i / samples for i in range(samples)]
        return ClosedPath((self.point_at(theta, offset) for theta in parameters),
                          (self.heading_at(theta, sign) for theta in parameters))

    def right_lane(self, samples=4096):
        """Reference path for the ego and its two slow leaders."""
        return self.path(self.right_offset, samples)

    def opposing_lane(self, samples=4096):
        """The other lane, traversed in its legal opposite direction."""
        return self.path(-self.right_offset, samples, direction=-self.sign)


def transform_point(pose, point):
    """Transform a body-frame point with an (x, y, yaw_rad) world pose."""
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c * point[0] - s * point[1], y + s * point[0] + c * point[1]
