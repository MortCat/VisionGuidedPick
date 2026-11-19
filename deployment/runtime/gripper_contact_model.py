"""Resolve planar pad contact centres from the deployed URDF/STL, before ROS starts.

The model descriptor identifies contact meshes and their inward-facing normals.
No object dimensions, live vision, ROS, USD or numerical offsets belong here.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import tempfile
import xml.etree.ElementTree as ET


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def add(a, b):
    return [x+y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def transform(m, p):
    return [dot(m[i][:3], p)+m[i][3] for i in range(3)]


def inverse(m):
    out = identity()
    for i in range(3):
        for j in range(3):
            out[i][j] = m[j][i]
        out[i][3] = -dot(out[i][:3], [m[j][3] for j in range(3)])
    return out


def origin(element):
    out = identity()
    if element is None:
        return out
    r, p, y = map(float, element.get('rpy', '0 0 0').split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    for i, value in enumerate(map(float, element.get('xyz', '0 0 0').split())):
        out[i][:3], out[i][3] = rotation[i], value
    return out


def triangles(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if 84+50*count == len(data):
        return [[list(struct.unpack_from('<3f', data, 84+50*i+12+12*j)) for j in range(3)] for i in range(count)]
    vertices = [list(map(float, v.split())) for v in re.findall(r'vertex\s+([^\r\n]+)', data.decode('ascii'))]
    if not vertices or len(vertices) % 3:
        raise ValueError('Invalid STL contact mesh: '+str(path))
    return [vertices[i:i+3] for i in range(0, len(vertices), 3)]


def planar_contact_center(path, inward, scale):
    length = math.sqrt(dot(inward, inward))
    if not length:
        raise ValueError('Contact normal must be nonzero')
    normal = [v/length for v in inward]
    faces = []
    for triangle in triangles(path):
        a, b, c = [[v*s for v, s in zip(p, scale)] for p in triangle]
        n = cross([x-y for x, y in zip(b, a)], [x-y for x, y in zip(c, a)])
        area = math.sqrt(dot(n, n))/2.
        if area == 0 or dot(n, normal)/(2.*area) < 1.-1e-6:
            continue
        center = [(a[i]+b[i]+c[i])/3. for i in range(3)]
        faces.append((dot(center, normal), area, center))
    if not faces:
        raise ValueError('No planar contact face matches the model normal: '+str(path))
    # Use the exposed contact plane, not recessed parallel faces of a stepped pad.
    outermost = max(face[0] for face in faces)
    selected = [face for face in faces if math.isclose(face[0], outermost, rel_tol=1e-7, abs_tol=1e-8)]
    total = sum(face[1] for face in selected)
    return [sum(area*center[i] for _, area, center in selected)/total for i in range(3)]


def derive_contact_center(urdf_path, tcp_link, contact_meshes):
    urdf_path = Path(urdf_path)
    robot = ET.parse(urdf_path).getroot()
    joints = {j.find('child').get('link'): j for j in robot.findall('joint')}
    poses = {}
    def link_pose(name):
        if name not in poses:
            joint = joints.get(name)
            poses[name] = identity() if joint is None else multiply(link_pose(joint.find('parent').get('link')), origin(joint.find('origin')))
        return poses[name]
    if tcp_link not in {l.get('name') for l in robot.findall('link')}:
        raise ValueError('Model TCP link is absent: '+tcp_link)
    tcp_inverse = inverse(link_pose(tcp_link))
    parts, centers = [], []
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            filename = mesh.get('filename', '')
            path = (urdf_path.parent/filename).resolve()
            spec = next((s for s in contact_meshes if path == (urdf_path.parent/s['mesh']).resolve()), None)
            if spec is None:
                continue
            center = planar_contact_center(path, spec['inward_normal_mesh'], list(map(float, mesh.get('scale', '1 1 1').split())))
            local = transform(multiply(tcp_inverse, multiply(link_pose(link.get('name')), origin(collision.find('origin')))), center)
            centers.append(local)
            parts.append({'link': link.get('name'), 'mesh': filename, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'center_tcp_m': local})
    if not centers:
        raise ValueError('No declared contact surfaces in '+str(urdf_path))
    center = [sum(p[i] for p in centers)/len(centers) for i in range(3)]
    return {'finger_center_tcp_m': [0. if abs(v) < 1e-9 else v for v in center], 'contacts': parts,
            'urdf_sha256': hashlib.sha256(urdf_path.read_bytes()).hexdigest(), 'tcp_link': tcp_link}


def write_atomic(path, text):
    with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=path.parent,
                                     prefix=path.name+'.', suffix='.tmp', delete=False) as stream:
        stream.write(text)
        temporary = Path(stream.name)
    try:
        temporary.replace(path)
    finally:
        temporary.unlink(missing_ok=True)


def resolve_contact_settings(directory):
    directory = Path(directory)
    descriptor = directory/'assets/gripper_contact_model.json'
    if not descriptor.exists():
        return {}
    model = json.loads(descriptor.read_text())
    resolved = {}
    for arm, item in model['arms'].items():
        measurement = derive_contact_center(directory/item['urdf'], item['tcp_link'], model['contact_meshes'])
        path = directory/item['settings']
        text = path.read_text()
        pattern = r'(?m)^  finger_center_tcp_m:\n(?:  - [^\n]+\n){3}'
        block = '  finger_center_tcp_m:\n'+''.join('  - '+format(v, '.12g')+'\n' for v in measurement['finger_center_tcp_m'])
        updated, count = re.subn(pattern, lambda m: block, text)
        if count != 1:
            raise ValueError('Expected one existing finger_center_tcp_m setting: '+str(path))
        if updated != text:
            # Source config is a generated calibration result, never modified in a control loop.
            write_atomic(path, updated)
        resolved[arm] = measurement
    output = descriptor.with_name('gripper_contact_model.resolved.json')
    serialized = json.dumps(resolved, indent=2)+'\n'
    if not output.exists() or output.read_text() != serialized:
        write_atomic(output, serialized)
    return resolved


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('settings_directory')
    args = parser.parse_args()
    print(json.dumps(resolve_contact_settings(args.settings_directory), indent=2))
