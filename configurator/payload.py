"""Existing 6DOF reference with URDF-derived variant/tool corrections.

The standalone geometry model is diagnostic, not the user-validated baseline.
New print masses use PETG 15%/1.6 mm shell and uniform effective density.
70% reach scales levers rather than solving an IK pose.
"""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET
import json

import numpy as np

MOTORS = {'STS3215': (30 * .0980665, .056),
          'STS3250': (50 * .0980665, .075),
          'ST3120': (120 * .0980665, .210)}


def xyz(value):
    return np.array([float(x) for x in (value or '0 0 0').split()])


def rotation(axis, angle):
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + np.sin(angle) * cross + (1 - np.cos(angle)) * (cross @ cross)


def origin(element):
    out = np.eye(4)
    if element is not None:
        roll, pitch, yaw = xyz(element.get('rpy'))
        out[:3, :3] = (rotation(np.array([0., 0., 1.]), yaw) @
                        rotation(np.array([0., 1., 0.]), pitch) @
                        rotation(np.array([1., 0., 0.]), roll))
        out[:3, 3] = xyz(element.get('xyz'))
    return out


@lru_cache(maxsize=128)
def mesh_vertices(path, modified):
    # trimesh is also used by the repository's print-mass estimator.
    import trimesh
    return np.asarray(trimesh.load(path, force='mesh', process=False).vertices)


def calculate_geometry(urdf, src: Path, args):
    root = ET.fromstring(urdf)
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    children = {name: [] for name in links}
    child_links = set()
    for joint in joints.values():
        parent, child = joint.find('parent').get('link'), joint.find('child').get('link')
        children[parent].append(joint)
        child_links.add(child)
    frames, pivots, axes, downstream = {}, {}, {}, {}
    positions = {'joint_elbow': np.pi}

    def walk(name, frame):
        frames[name] = frame
        descendants = {name}
        for joint in children[name]:
            jname = joint.get('name')
            jf = frame @ origin(joint.find('origin'))
            motion = np.eye(4)
            if joint.get('type') not in ('fixed',):
                axis_el = joint.find('axis')
                axis = xyz(axis_el.get('xyz') if axis_el is not None else '1 0 0')
                pivots[jname], axes[jname] = jf[:3, 3], jf[:3, :3] @ axis
                motion[:3, :3] = rotation(axis, positions.get(jname, 0.))
            child = joint.find('child').get('link')
            desc = walk(child, jf @ motion)
            downstream[jname] = desc
            descendants.update(desc)
        return descendants

    walk(next(iter(set(links) - child_links)), np.eye(4))
    masses, centers = {}, {}
    selected = {'servo_shoulder_1': args.get('shoulder_motor', 'STS3250'),
                'servo_elbow_1': args.get('elbow_motor', 'STS3250')}
    for name, link in links.items():
        inertial = link.find('inertial')
        if inertial is None:
            continue
        masses[name] = float(inertial.find('mass').get('value'))
        if name in selected:
            if selected[name] not in MOTORS:
                raise ValueError('Unknown actuator model')
            masses[name] = MOTORS[selected[name]][1]
        centers[name] = (frames[name] @ origin(inertial.find('origin')))[:3, 3]

    flange = frames['wrist_flange']
    inv_flange = np.linalg.inv(flange)
    tool_desc = set().union(*(downstream[j.get('name')] for j in children['wrist_flange'])) if children['wrist_flange'] else set()
    # Conservative distal visual extent is used until a tool authors a TCP.
    # Works for each package's geometry, without a hardcoded gripper mass/reach.
    distal = 0.
    for name in tool_desc:
        for visual in links[name].findall('visual'):
            geometry = visual.find('geometry')
            mesh = geometry.find('mesh')
            vertices = None
            if mesh is not None:
                uri = mesh.get('filename')
                if uri.startswith('/pkg/'):
                    pkg, relative = uri[len('/pkg/'):].split('/', 1)
                    path = src / pkg / relative
                    vertices = mesh_vertices(str(path), path.stat().st_mtime_ns) * xyz(mesh.get('scale', '1 1 1'))
            box = geometry.find('box')
            if box is not None:
                h = xyz(box.get('size')) / 2
                vertices = np.array([[x,y,z] for x in (-h[0],h[0]) for y in (-h[1],h[1]) for z in (-h[2],h[2])])
            if vertices is not None:
                f = inv_flange @ frames[name] @ origin(visual.find('origin'))
                distal = max(distal, float(np.max(vertices @ f[0, :3] + f[0, 3])))
    payload_point = (flange @ np.array([distal, 0, 0, 1]))[:3]
    shoulder = pivots['joint_shoulder']
    reach = float(np.linalg.norm((payload_point - shoulder)[:2]))
    joint_models = {'joint_shoulder': args.get('shoulder_motor', 'STS3250'),
                    'joint_elbow': args.get('elbow_motor', 'STS3250'),
                    'joint_wrist_tilt': 'STS3215', 'joint_wrist_roll': 'STS3215'}
    if 'joint_wrist_yaw' in joints:
        joint_models['joint_wrist_yaw'] = 'STS3215'
    gravity = np.array([0., 0., -9.81])
    moments = {}
    for joint, model in joint_models.items():
        pivot, axis = pivots[joint], axes[joint]
        self_moment = abs(sum(float(np.dot(axis, np.cross(centers[name] - pivot, gravity * masses[name])))
                              for name in downstream[joint] if name in masses))
        lever = abs(float(np.dot(axis, np.cross(payload_point - pivot, gravity))))
        moments[joint] = {'self_torque_nm': self_moment, 'payload_torque_nm_per_kg': lever,
                         'stall_torque_nm': MOTORS[model][0]}
    out = {}
    for regime, factor in [('duty', .7), ('hold', .5), ('peak', 1.)]:
        for tag, ratio in [('full', 1.), ('70', .7)]:
            capacities = {joint: (m['stall_torque_nm'] * factor - m['self_torque_nm'] * ratio) /
                          (m['payload_torque_nm_per_kg'] * ratio)
                          for joint, m in moments.items() if m['payload_torque_nm_per_kg'] > 1e-8}
            binding = min(capacities, key=capacities.get)
            out[f'{regime}-{tag}'] = {'payload': capacities[binding], 'joint': binding,
                                     'ps': capacities['joint_shoulder'], 'pe': capacities['joint_elbow']}
    return {'out': out, 'reach': reach, 'mTotal': sum(masses[n] for n in downstream['joint_shoulder'] if n in masses),
            'moments': moments, 'tool_offset_m': distal,
            'assumptions': 'PETG 1.27 g/cm³, 15% infill, 1.6 mm shell; uniform effective density; one actuator per joint; horizontal extended pose; 70% scales levers; tool distal visual extent is provisional TCP.'}


def calculate(urdf, src: Path, args, reference_urdf):
    """Preserve the existing 6DOF/jaws baseline; apply CAD-derived differences.

    Absolute URDF gravity predictions are a different, unvalidated model.
    Comparing both variants at identical rail lengths and actuator selections
    isolates the extra actuator, printed parts and changed tool load point.
    """
    geometry = calculate_geometry(urdf, src, args)
    reference = calculate_geometry(reference_urdf, src, args)
    baseline = json.loads(Path(__file__).with_name('payload_reference.json').read_text())
    root = ET.fromstring(urdf)
    def rail_length(link):
        return float(root.find(f"link[@name='{link}']/visual/geometry/box").get('size').split()[0])
    ls, le = rail_length('arm_extrusion_1'), rail_length('forearm_extrusion_1')
    sh, el = args.get('shoulder_motor', 'STS3250'), args.get('elbow_motor', 'STS3250')
    l1 = baseline['shoulder_fixed_m'] + ls + (.015 if sh == 'ST3120' else 0)
    l2 = baseline['elbow_fixed_m'] + le + (.030 if el == 'ST3120' else 0)
    w = baseline['wrist_tool_m']
    prints = baseline['printed_groups_kg']
    ms = baseline['rail_mass_multiplier'] * .46 * ls
    me = baseline['rail_mass_multiplier'] * .46 * le
    mel = prints['elbow'] + baseline['pitch_servo_mass_multiplier'] * MOTORS[el][1]
    mw = prints['wrist'] + prints['gripper'] + 3 * MOTORS['STS3215'][1] + .0166
    brackets = {
        'joint_shoulder': ms*l1/2 + mel*l1 + me*(l1+l2/2) + mw*(l1+l2+w/2),
        'joint_elbow': me*l2/2 + mw*(l2+w/2),
    }
    levers = {'joint_shoulder': l1+l2+w, 'joint_elbow': l2+w}
    capacity = {'joint_shoulder': MOTORS[sh][0]*baseline['pitch_capacity_multiplier'],
                'joint_elbow': MOTORS[el][0]*baseline['pitch_capacity_multiplier']}
    # Changes are measured in the same URDF pose, so unchanged 6DOF/jaws
    # components cancel exactly instead of silently changing its baseline.
    for joint in brackets:
        current, old = geometry['moments'][joint], reference['moments'][joint]
        brackets[joint] += (current['self_torque_nm'] - old['self_torque_nm']) / 9.81
        levers[joint] += (current['payload_torque_nm_per_kg'] - old['payload_torque_nm_per_kg']) / 9.81
    out = {}
    for regime, factor in [('duty', .7), ('hold', .5), ('peak', 1.)]:
        for tag, ratio in [('full', 1.), ('70', .7)]:
            caps = {joint: (capacity[joint]*factor/(9.81*ratio)-brackets[joint])/levers[joint]
                    for joint in brackets}
            binding = min(caps, key=caps.get)
            out[f'{regime}-{tag}'] = {'payload': caps[binding], 'joint': binding,
                                     'ps': caps['joint_shoulder'], 'pe': caps['joint_elbow']}
    return {**geometry, 'out': out,
            'reach': levers['joint_shoulder'],
            'mTotal': ms+me+mel+mw + geometry['mTotal'] - reference['mTotal'],
            'model': 'Existing 6DOF/jaws reference plus CAD-derived mass and lever corrections',
            'geometry_moments': geometry['moments'],
            'moments': {joint: {'self_torque_nm': brackets[joint]*9.81,
                                'payload_torque_nm_per_kg': levers[joint]*9.81,
                                'reference_capacity_nm': capacity[joint]}
                        for joint in brackets},
            'assumptions': 'Original 6DOF reference capacity and lumped weights retained. New wrist corrections: STS3215 56g, PETG 15% infill, 1.6mm shell. 7DOF is an extrapolation, not a bench-tested rated payload. 70% scales levers.'}
