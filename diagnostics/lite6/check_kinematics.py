"""Evaluate the generated URDF offline; no ROS nodes, sockets, or arm commands."""
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np


def rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle)*skew + (1-np.cos(angle))*(skew @ skew)


def chain(urdf):
    joints = {j.find('child').get('link'): j for j in ET.parse(urdf).getroot().findall('joint')}
    result, link = [], 'link_eef'
    while link in joints:
        joint = joints[link]
        result.append(joint)
        link = joint.find('parent').get('link')
    return list(reversed(result))


def evaluate(joints, angles):
    transform = np.eye(4)
    axes, origins = [], []
    for joint in joints:
        origin = joint.find('origin')
        xyz = np.zeros(3) if origin is None else np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        rpy = np.zeros(3) if origin is None else np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        local = np.eye(4)
        local[:3, 3] = xyz
        local[:3, :3] = (rotation(np.array([0.,0.,1.]), rpy[2]) @
                          rotation(np.array([0.,1.,0.]), rpy[1]) @
                          rotation(np.array([1.,0.,0.]), rpy[0]))
        transform = transform @ local
        if joint.get('type') in ('revolute', 'continuous'):
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axes.append(transform[:3, :3] @ axis)
            origins.append(transform[:3, 3].copy())
            move = np.eye(4)
            move[:3, :3] = rotation(axis, angles[joint.get('name')])
            transform = transform @ move
    jacobian = np.array([np.r_[np.cross(a, transform[:3, 3]-p), a]
                         for a, p in zip(axes, origins)]).T
    return transform, jacobian


def main():
    joints = chain(sys.argv[1])
    results = []
    for name, degrees in [('all_zero', [0]*6),
                           ('requested_initial_pose', [0, 9.93, 31.8, 0, 21.87, 0])]:
        angles = {f'joint{i+1}': np.deg2rad(v) for i, v in enumerate(degrees)}
        transform, jacobian = evaluate(joints, angles)
        numerical = np.zeros((6, 6))
        delta = 1e-7
        for i in range(6):
            perturbed = dict(angles)
            perturbed[f'joint{i+1}'] += delta
            other, _ = evaluate(joints, perturbed)
            numerical[:3, i] = (other[:3, 3]-transform[:3, 3])/delta
            omega = ((other[:3, :3]-transform[:3, :3])/delta) @ transform[:3, :3].T
            numerical[3:, i] = [omega[2,1], omega[0,2], omega[1,0]]
        error = float(np.max(np.abs(numerical-jacobian)))
        assert error < 1e-6, error
        singular = np.linalg.svd(jacobian, compute_uv=False)
        results.append(dict(pose=name, degrees=degrees,
                            singular_values=singular.tolist(),
                            condition_number=float(singular[0]/singular[-1]),
                            rank=int(np.linalg.matrix_rank(jacobian)),
                            finite_difference_max_error=error))
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
