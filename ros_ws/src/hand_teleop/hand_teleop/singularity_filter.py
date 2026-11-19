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
    zone: str = 'STOP'
    speed_scale: float = 0.0
    predicted_condition: float = None


@dataclass(frozen=True)
class MotionPrediction:
    conditions: tuple
    joint_margin: float
    speed_usage: float
    reason: str = ''


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
    """Preview and scale whole commands with gated singularity recovery.

    This conservative UI prefilter supplements Servo. It is not a path planner
    or a guarantee about a physical robot's response.
    """

    def __init__(self, model, warning=17.0, halt=25.0, horizon=0.3, step=0.05,
                 hysteresis=0.5, recovery_speed=0.005, recovery_ceiling=28.0):
        if (not all(map(math.isfinite, (warning, halt, horizon, step, hysteresis)))
                or not 1 < warning < halt <= 25 or not 0 < step <= horizon <= 0.5):
            raise ValueError('Invalid singularity filter configuration')
        if not 0 <= hysteresis < min(warning - 1, halt - warning):
            raise ValueError('Invalid zone hysteresis')
        if (not math.isfinite(recovery_speed) or not 0 < recovery_speed <= 0.01
                or not math.isfinite(recovery_ceiling)
                or not halt < recovery_ceiling <= 28.):
            raise ValueError('Invalid recovery bounds')
        self.model, self.warning, self.halt = model, warning, halt
        self.recovery_speed, self.recovery_ceiling = recovery_speed, recovery_ceiling
        self.hysteresis = hysteresis
        self.zone = 'NORMAL'
        self.steps = math.ceil(horizon/step)
        self.dt = horizon/self.steps

    def reset(self):
        self.zone = 'NORMAL'

    def classify(self, condition):
        if condition >= self.halt:
            self.zone = 'RECOVERY'
        elif self.zone == 'RECOVERY' and condition >= self.halt - self.hysteresis:
            pass
        elif condition >= self.warning:
            self.zone = 'CAUTION'
        elif self.zone != 'NORMAL' and condition >= self.warning - self.hysteresis:
            self.zone = 'CAUTION'
        else:
            self.zone = 'NORMAL'
        return self.zone

    def approach_scale(self, condition):
        fraction = np.clip((self.halt - condition) / (self.halt - self.warning), 0., 1.)
        return float(fraction * fraction * (3. - 2. * fraction))

    @staticmethod
    def condition(jacobian):
        singular = np.linalg.svd(jacobian, compute_uv=False)
        if not np.isfinite(singular).all() or singular[-1] <= 1e-8:
            return math.inf
        return float(singular[0]/singular[-1])

    def predict(self, velocity, angles):
        """Preview the complete velocity vector without applying zone policy."""
        angles = np.array(angles, dtype=float, copy=True)
        twist = np.r_[velocity, [0., 0., 0.]]
        conditions = []
        margin = float(np.min(np.minimum(
            angles - self.model.lower, self.model.upper - angles)))
        speed_usage = 0.0
        for _ in range(self.steps):
            jacobian = self.model.jacobian(angles)
            left, singular, right = np.linalg.svd(jacobian, full_matrices=False)
            weights = singular / (singular * singular + 1e-6)
            joint_speed = right.T @ (weights * (left.T @ twist))
            if not np.isfinite(joint_speed).all():
                return MotionPrediction(tuple(conditions), margin, speed_usage,
                                        'Cannot predict motion reliably')
            speed_usage = max(speed_usage, float(np.max(
                np.abs(joint_speed) / self.model.speeds)))
            if speed_usage > 1.0:
                return MotionPrediction(tuple(conditions), margin, speed_usage,
                                        'Motion would exceed a joint speed limit')
            angles += joint_speed * self.dt
            margin = min(margin, float(np.min(np.minimum(
                angles - self.model.lower, self.model.upper - angles))))
            if margin < 0.1:
                return MotionPrediction(tuple(conditions), margin, speed_usage,
                                        'Motion would approach a configured joint limit')
            conditions.append(self.condition(self.model.jacobian(angles)))
        return MotionPrediction(tuple(conditions), margin, speed_usage)

    def filter(self, velocity, positions, recovery_enabled=False):
        condition = None
        predicted = None
        zone = 'STOP'

        def blocked(reason):
            return MotionDecision((0.0,0.0,0.0), True, reason, condition,
                                  zone, 0.0, predicted)

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
            zone = self.classify(condition)
            if not math.isfinite(condition) or condition >= self.recovery_ceiling:
                return blocked('Near singularity: reposition required')
            if np.any(angles < self.model.lower + 0.1) or np.any(angles > self.model.upper - 0.1):
                return blocked('Pose is too close to a configured joint limit')
            if zone == 'RECOVERY' and not recovery_enabled:
                return blocked('Near singularity: recovery enable and fresh hold required')
            if np.linalg.norm(requested) == 0:
                return MotionDecision((0.,0.,0.), False, 'Ready', condition,
                                      zone, 1.0, condition)
            if zone == 'RECOVERY':
                scale = float(min(1.0, self.recovery_speed / np.linalg.norm(requested)))
                candidate = requested * scale
                prediction = self.predict(candidate, angles)
                if prediction.reason:
                    return blocked(prediction.reason)
                previous = condition
                for predicted in prediction.conditions:
                    if not math.isfinite(predicted) or predicted >= previous - 1e-5:
                        return blocked('Recovery must improve conditioning at every preview step')
                    previous = predicted
                return MotionDecision(tuple(float(value) for value in candidate), False,
                                      'Recovery direction improves conditioning', condition,
                                      zone, scale, max(prediction.conditions))
            prediction = self.predict(requested, angles)
            if prediction.reason:
                return blocked(prediction.reason)
            peak = max(prediction.conditions)
            approaching = peak > condition + 1e-5
            scale = self.approach_scale(max(condition, peak)) if approaching else 1.0
            if approaching and scale == 0.0:
                scale = 0.5 * self.approach_scale(condition)
            for _ in range(12):
                if scale < 1e-4:
                    break
                candidate = requested * scale
                prediction = self.predict(candidate, angles)
                if prediction.reason:
                    return blocked(prediction.reason)
                predicted = max(prediction.conditions)
                if math.isfinite(predicted) and predicted < self.halt:
                    return MotionDecision(tuple(float(value) for value in candidate),
                                          False, 'Slowing near singularity' if scale < 1.0
                                          else 'Direction allowed', condition,
                                          'CAUTION' if scale < 1.0 else zone,
                                          scale, predicted)
                scale *= 0.5
            return blocked('Motion would approach a singularity; reverse direction')
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError, FloatingPointError):
            return blocked('Robot pose or motion prediction is unavailable')
