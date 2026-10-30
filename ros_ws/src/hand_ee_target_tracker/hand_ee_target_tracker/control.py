"""
Control law for ee_target_tracker. No ROS imports, so it can be tested anywhere.

Poses are a position (x, y, z) in metres and a quaternion (x, y, z, w), both in
``link_base``. See docs/interfaces/ee_target_tracker.md for the mapping.
"""
import math
from dataclasses import dataclass
from typing import Tuple

Vec3 = Tuple[float, float, float]
Quat = Tuple[float, float, float, float]

ZERO: Vec3 = (0.0, 0.0, 0.0)
_SMALL_ANGLE = 1e-9


@dataclass(frozen=True)
class Limits:
    """Tracker limits, named as the ROS parameters in the contract."""

    publish_rate: float = 50.0
    max_speed: float = 0.05
    max_angular_speed: float = 0.5
    gain: float = 1.0
    angular_gain: float = 1.0
    deadband: float = 0.002
    position_tolerance: float = 0.005
    orientation_tolerance: float = 0.02


@dataclass(frozen=True)
class Command:
    """One cycle of output: a bounded twist and the setpoint one step ahead."""

    linear: Vec3
    angular: Vec3
    distance: float
    angle: float
    setpoint_position: Vec3
    setpoint_orientation: Quat


def norm(v):
    """Euclidean length of a vector."""
    return math.sqrt(sum(c * c for c in v))


def scale(v, s):
    """Multiply a vector by a scalar."""
    return tuple(c * s for c in v)


def is_finite(values):
    """Return True if every number is finite (no NaN or infinity)."""
    return all(math.isfinite(c) for c in values)


def normalize_quat(q):
    """Return ``q`` with unit length, or None if it is not a usable orientation."""
    if not is_finite(q):
        return None
    length = norm(q)
    if length < 1e-6:
        return None
    return tuple(c / length for c in q)


def quat_multiply(a, b):
    """Hamilton product ``a * b`` (apply ``b`` first, then ``a``)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_conjugate(q):
    """Inverse of a unit quaternion."""
    return (-q[0], -q[1], -q[2], q[3])


def rotation_vector(q_from, q_to):
    """
    Return axis times angle of the shortest rotation taking ``q_from`` to ``q_to``.

    The vector is expressed in the parent frame (``link_base``), so it can be used
    directly as an angular velocity direction there.
    """
    x, y, z, w = quat_multiply(q_to, quat_conjugate(q_from))
    if w < 0.0:  # q and -q are the same rotation; take the short way round
        x, y, z, w = -x, -y, -z, -w
    sin_half = norm((x, y, z))
    if sin_half < _SMALL_ANGLE:
        return (2.0 * x, 2.0 * y, 2.0 * z)
    angle = 2.0 * math.atan2(sin_half, w)
    factor = angle / sin_half
    return (x * factor, y * factor, z * factor)


def quat_from_rotation_vector(rotvec):
    """Return the unit quaternion for a rotation given as axis times angle."""
    angle = norm(rotvec)
    if angle < _SMALL_ANGLE:
        return (0.0, 0.0, 0.0, 1.0)
    s = math.sin(angle / 2.0) / angle
    return (rotvec[0] * s, rotvec[1] * s, rotvec[2] * s, math.cos(angle / 2.0))


def in_workspace(position, lower, upper):
    """Return True if ``position`` is inside the axis-aligned box, edges included."""
    return all(lo <= p <= hi for p, lo, hi in zip(position, lower, upper))


def is_reached(distance, angle, limits):
    """Return True if both errors are within their tolerance."""
    return (distance <= limits.position_tolerance
            and angle <= limits.orientation_tolerance)


def compute_command(current_position, current_orientation,
                    goal_position, goal_orientation, limits):
    """
    Return a bounded twist toward the goal, plus the setpoint one cycle ahead.

    Position and orientation are limited independently, so a large rotation never
    inflates the translation speed. ``current_orientation`` and
    ``goal_orientation`` must be unit quaternions.
    """
    error = tuple(g - c for g, c in zip(goal_position, current_position))
    distance = norm(error)
    if distance < limits.deadband:  # total error, not per axis: no creeping along one axis
        linear = ZERO
    else:
        speed = min(max(limits.gain * distance, 0.0), limits.max_speed)
        linear = scale(error, speed / distance)

    rotvec = rotation_vector(current_orientation, goal_orientation)
    angle = norm(rotvec)
    angular = scale(rotvec, limits.angular_gain)
    angular_speed = norm(angular)
    if angular_speed > limits.max_angular_speed:
        angular = scale(angular, limits.max_angular_speed / angular_speed)

    dt = 1.0 / limits.publish_rate
    setpoint_position = tuple(c + v * dt for c, v in zip(current_position, linear))
    step = quat_from_rotation_vector(scale(angular, dt))
    setpoint_orientation = normalize_quat(quat_multiply(step, current_orientation))
    return Command(linear, angular, distance, angle,
                   setpoint_position, setpoint_orientation)


def hold_command(current_position, current_orientation):
    """Zero twist with the setpoint at the current pose: the arm holds still."""
    return Command(ZERO, ZERO, 0.0, 0.0, tuple(current_position),
                   tuple(current_orientation))
