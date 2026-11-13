"""Predict mouse-command effects from the configured URDF, without moving a robot."""

from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

import numpy as np


@dataclass(frozen=True)
class MotionDecision:
    velocity: tuple
    blocked: bool
    reason: str
    condition: float = None


def fresh_feedback(stamp_ns, received_wall, ros_now_ns, wall_now, timeout=0.2):
    return (stamp_ns is not None and received_wall is not None
            and 0 <= ros_now_ns-stamp_ns <= int(timeout*1e9)
            and 0 <= wall_now-received_wall <= timeout)


def joint_positions(names, values):
    if len(names) != len(values) or len(names) != len(set(names)):
        raise ValueError('Invalid joint feedback layout')
    if not all(math.isfinite(value) for value in values):
        raise ValueError('Invalid joint feedback values')
    return dict(zip(names, values))


def rotation(axis, angle):
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)


def vector(text, default):
    result = np.fromstring(text if text is not None else default, sep=' ')
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError('Invalid robot model geometry')
    return result


class SerialModel:
    """The configured base-to-tool chain, including its existing joint bounds."""

    def __init__(self, description, base='link_base', tip='link_eef'):
        try:
            root = ET.fromstring(description)
        except ET.ParseError as error:
            raise ValueError('Robot model is not valid XML') from error
        links = {link.get('name') for link in root.findall('link')}
        if base not in links or tip not in links:
            raise ValueError('Robot model does not contain the command frames')
        parents = {}
        for joint in root.findall('joint'):
            child = joint.find('child')
            if child is None or child.get('link') in parents:
                raise ValueError('Invalid robot model tree')
            parents[child.get('link')] = joint
        ordered, seen, link = [], set(), tip
        while link != base:
            if link in seen or link not in parents:
                raise ValueError('Robot model has no valid base-to-tool chain')
            seen.add(link)
            joint = parents[link]
            parent = joint.find('parent')
            if parent is None:
                raise ValueError('Robot model joint has no parent')
            ordered.append(joint)
            link = parent.get('link')
        self.segments, self.names = [], []
        self.lower, self.upper, self.speeds = [], [], []
        for joint in reversed(ordered):
            kind = joint.get('type')
            if kind not in ('fixed', 'revolute', 'continuous') or joint.find('mimic') is not None:
                raise ValueError('Unsupported robot model joint')
            origin = joint.find('origin')
            xyz = vector(None if origin is None else origin.get('xyz'), '0 0 0')
            rpy = vector(None if origin is None else origin.get('rpy'), '0 0 0')
            transform = np.eye(4)
            transform[:3, 3] = xyz
            transform[:3, :3] = (rotation(np.array([0.,0.,1.]), rpy[2]) @
                                 rotation(np.array([0.,1.,0.]), rpy[1]) @
                                 rotation(np.array([1.,0.,0.]), rpy[0]))
            index, axis = None, None
            if kind != 'fixed':
                axis_element = joint.find('axis')
                axis = vector(None if axis_element is None else axis_element.get('xyz'), '1 0 0')
                length = np.linalg.norm(axis)
                if length < 1e-12:
                    raise ValueError('Robot model has a zero joint axis')
                axis /= length
                limit = joint.find('limit')
                try:
                    lower = -math.inf if kind == 'continuous' else float(limit.get('lower'))
                    upper = math.inf if kind == 'continuous' else float(limit.get('upper'))
                    speed = float(limit.get('velocity'))
                except (AttributeError, TypeError, ValueError) as error:
                    raise ValueError('Robot model has incomplete joint limits') from error
                if ((kind != 'continuous' and not all(map(math.isfinite, (lower, upper))))
                        or lower >= upper or not math.isfinite(speed) or speed <= 0):
                    raise ValueError('Robot model has invalid joint limits')
                index = len(self.names)
                self.names.append(joint.get('name'))
                self.lower.append(lower); self.upper.append(upper); self.speeds.append(speed)
            self.segments.append((transform, axis, index))
        if len(self.names) != 6 or len(set(self.names)) != 6 or None in self.names:
            raise ValueError('Mouse singularity filter requires a six-joint chain')
        self.lower = np.array(self.lower)
        self.upper = np.array(self.upper)
        self.speeds = np.array(self.speeds)

    def jacobian(self, angles):
        transform = np.eye(4)
        axes, origins = [], []
        for origin, axis, index in self.segments:
            transform = transform @ origin
            if index is not None:
                axes.append(transform[:3,:3] @ axis)
                origins.append(transform[:3,3].copy())
                motion = np.eye(4)
                motion[:3,:3] = rotation(axis, angles[index])
                transform = transform @ motion
        return np.array([np.r_[np.cross(axis, transform[:3,3]-position), axis]
                         for axis, position in zip(axes, origins)]).T


class SingularityFilter:
    """Reject the complete drag when its predicted path approaches a singularity.

    This conservative UI prefilter supplements Servo. It is not a path planner
    or a guarantee about a physical robot's response.
    """

    def __init__(self, model, warning=17.0, halt=25.0, horizon=0.3, step=0.05):
        if (not all(map(math.isfinite, (warning, halt, horizon, step)))
                or not 1 < warning < halt <= 25 or not 0 < step <= horizon <= 0.5):
            raise ValueError('Invalid singularity filter configuration')
        self.model, self.warning, self.halt = model, warning, halt
        self.steps = math.ceil(horizon/step)
        self.dt = horizon/self.steps

    @staticmethod
    def condition(jacobian):
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if not np.isfinite(singular).all() or singular[-1] <= 1e-8:
            return math.inf
        return float(singular[0]/singular[-1])

    def filter(self, velocity, positions):
        condition = None

        def blocked(reason):
            return MotionDecision((0.0,0.0,0.0), True, reason, condition)

        try:
            requested = np.asarray(velocity, dtype=float)
            if requested.shape != (3,) or not np.isfinite(requested).all():
                return blocked('Invalid mouse velocity')
            angles = np.array([positions[name] for name in self.model.names], dtype=float)
            if not np.isfinite(angles).all():
                return blocked('Invalid joint feedback')
            outside = np.where((angles < self.model.lower) | (angles > self.model.upper))[0]
            if len(outside):
                return blocked(self.model.names[outside[0]] + ' is outside configured joint limits')
            jacobian = self.model.jacobian(angles)
            condition = self.condition(jacobian)
            if condition >= self.halt:
                return blocked('Arm is too close to a singularity; reposition before mouse control')
            if np.linalg.norm(requested) == 0:
                return MotionDecision((0.,0.,0.), False, 'Ready', condition)
            twist = np.r_[requested, [0.,0.,0.]]
            previous = condition
            for _ in range(self.steps):
                u, singular, vt = np.linalg.svd(jacobian, full_matrices=False)
                # Damping prevents an inverse singular value from becoming infinite.
                weights = singular/(singular*singular + 1e-6)
                joint_speed = vt.T @ (weights*(u.T @ twist))
                if not np.isfinite(joint_speed).all():
                    return blocked('Cannot predict this drag reliably')
                if np.any(np.abs(joint_speed) > self.model.speeds):
                    return blocked('Drag would exceed a joint speed limit')
                angles = angles + joint_speed*self.dt
                if np.any(angles < self.model.lower+0.1) or np.any(angles > self.model.upper-0.1):
                    return blocked('Drag would approach a configured joint limit')
                jacobian = self.model.jacobian(angles)
                predicted = self.condition(jacobian)
                if predicted >= self.halt:
                    return blocked('Drag would approach a singularity')
                if predicted >= self.warning and predicted > previous+1e-5:
                    return blocked('Drag would move closer to a singularity; reverse direction')
                previous = predicted
            return MotionDecision(tuple(float(v) for v in requested), False, 'Direction allowed', condition)
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError, FloatingPointError):
            return blocked('Robot pose or motion prediction is unavailable')
