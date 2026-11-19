"""Fixed finger-spacer geometry shared by expanded and runtime robot models."""
import math
import xml.etree.ElementTree as ET


def apply_finger_spacer(urdf, geometry):
    if not geometry:
        return
    length = float(geometry['length_m'])
    width, depth = map(float, geometry['size_xy_m'])
    width_scale = float(geometry.get('width_scale', 1.))
    if not all(math.isfinite(v) and v > 0. for v in (length, width, depth, width_scale)):
        raise ValueError('finger spacer dimensions must be finite and positive')
    for link in urdf.findall('link'):
        name = link.get('name', '').rsplit('/', 1)[-1]
        if name not in ('panda_leftfinger', 'panda_rightfinger'):
            continue
        sign = 1. if name == 'panda_leftfinger' else -1.
        for element in list(link):
            if element.tag not in ('visual', 'collision'):
                continue
            origin = element.find('origin')
            if origin is None:
                origin = ET.SubElement(element, 'origin', xyz='0 0 0', rpy='0 0 0')
            xyz = list(map(float, origin.get('xyz', '0 0 0').split()))
            xyz[2] += length
            origin.set('xyz', ' '.join(map(str, xyz)))
            mesh = element.find('geometry/mesh')
            if mesh is not None:
                scale = list(map(float, mesh.get('scale', '1 1 1').split()))
                scale[0] *= width_scale
                mesh.set('scale', ' '.join(map(str, scale)))
        for kind in ('visual', 'collision'):
            element = ET.SubElement(link, kind, name='finger_spacer')
            ET.SubElement(element, 'origin', xyz=f'0 {sign*depth/2.} {length/2.}', rpy='0 0 0')
            shape = ET.SubElement(element, 'geometry')
            ET.SubElement(shape, 'box', size=f'{width} {depth} {length}')
