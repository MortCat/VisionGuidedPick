"""Model changes must move the derived contact centre; no simulator is required."""
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
from gripper_contact_model import derive_contact_center, planar_contact_center


MESH = Path(__file__).resolve().parents[2]/'config/factory_demo/assets/meshes/collision/finger_pad_a.stl'
CONTACTS = [{'mesh': str(MESH), 'inward_normal_mesh': [0, -1, 0]}]


def fixture(path, tcp_height=.1034, pad_scale=1.):
    robot = ET.Element('robot', name='fixture')
    for name in ['world', 'hand', 'left', 'right', 'tcp']:
        ET.SubElement(robot, 'link', name=name)
    def joint(name, parent, child, xyz, rpy='0 0 0'):
        j = ET.SubElement(robot, 'joint', name=name, type='fixed')
        ET.SubElement(j, 'parent', link=parent)
        ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz=xyz, rpy=rpy)
    joint('mount', 'world', 'hand', '2 3 4', '.2 -.3 .4')
    joint('left_fixed', 'hand', 'left', '0 .04 .0584')
    joint('right_fixed', 'hand', 'right', '0 -.04 .0584')
    joint('tcp_fixed', 'hand', 'tcp', '0 0 '+str(tcp_height))
    for name, yaw in [('left', 0.), ('right', math.pi)]:
        link = next(l for l in robot.findall('link') if l.get('name') == name)
        collision = ET.SubElement(link, 'collision')
        ET.SubElement(collision, 'origin', rpy='0 0 '+str(yaw))
        shape = ET.SubElement(collision, 'geometry')
        ET.SubElement(shape, 'mesh', filename=str(MESH), scale='1 1 '+str(pad_scale))
    ET.ElementTree(robot).write(path)


class ContactGeometryTest(unittest.TestCase):
    def test_transformed_model_matches_pad_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'robot.urdf';fixture(path)
            out = derive_contact_center(path, 'tcp', CONTACTS)
            self.assertEqual(len(out['contacts']), 2)
            self.assertAlmostEqual(out['finger_center_tcp_m'][0], 0., places=7)
            self.assertAlmostEqual(out['finger_center_tcp_m'][1], 0., places=7)
            self.assertAlmostEqual(out['finger_center_tcp_m'][2], .0584+.04-.1034, places=7)

    def test_tcp_change_is_derived_instead_of_reusing_offset(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'robot.urdf';fixture(path)
            before = derive_contact_center(path, 'tcp', CONTACTS)['finger_center_tcp_m']
            fixture(path, tcp_height=.1234)
            after = derive_contact_center(path, 'tcp', CONTACTS)['finger_center_tcp_m']
            self.assertAlmostEqual(after[2]-before[2], -.02, places=7)

    def test_changed_mesh_scale_changes_contact_center(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'robot.urdf';fixture(path)
            before = derive_contact_center(path, 'tcp', CONTACTS)['finger_center_tcp_m']
            fixture(path, pad_scale=1.5)
            after = derive_contact_center(path, 'tcp', CONTACTS)['finger_center_tcp_m']
            self.assertAlmostEqual(after[2]-before[2], .04*.5, places=7)

    def test_ascii_contact_patch(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'pad.stl'
            path.write_text('solid pad\nfacet normal 0 -1 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 0 1\nendloop\nendfacet\nendsolid pad\n')
            p = planar_contact_center(path, [0,-1,0], [1,1,1])
            self.assertAlmostEqual(p[0], 1./3.)
            self.assertAlmostEqual(p[2], 1./3.)
            with self.assertRaises(ValueError):
                planar_contact_center(path, [0,1,0], [1,1,1])

    def test_recessed_parallel_faces_do_not_shift_contact_center(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'pad.stl'
            triangles = []
            for y in [0., -.01]:
                triangles.append('facet normal 0 -1 0\nouter loop\nvertex 0 '+str(y)+' 0\nvertex 1 '+str(y)+' 0\nvertex 0 '+str(y)+' 1\nendloop\nendfacet\n')
            path.write_text('solid pad\n'+''.join(triangles)+'endsolid pad\n')
            center = planar_contact_center(path, [0,-1,0], [1,1,1])
            self.assertAlmostEqual(center[1], -.01)


if __name__ == '__main__':
    unittest.main()
