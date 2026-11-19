import unittest
import xml.etree.ElementTree as ET
from gripper_geometry import apply_finger_spacer


class FingerGeometryTest(unittest.TestCase):
    def model(self):
        return ET.fromstring('''<robot name="test">
          <link name="arm_01/panda_leftfinger"><inertial><mass value="0.014"/></inertial>
            <visual><geometry><mesh filename="finger.dae"/></geometry></visual>
            <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="finger.stl"/></geometry></collision></link>
          <link name="arm_01/panda_rightfinger"><collision><origin xyz="0 0 0" rpy="0 0 3.141592653589793"/>
            <geometry><mesh filename="finger.stl"/></geometry></collision></link>
          <joint name="finger"><limit lower="0" upper="0.04"/></joint></robot>''')

    def test_baseline_is_unchanged_without_geometry_asset(self):
        model = self.model(); before = ET.tostring(model)
        apply_finger_spacer(model, None)
        self.assertEqual(ET.tostring(model), before)

    def test_extension_and_width_preserve_closing_axis_mass_and_joint_limits(self):
        model = self.model()
        apply_finger_spacer(model, {'length_m': .01, 'size_xy_m': [.0252,.0264], 'width_scale': 1.2})
        for link in model.findall('link'):
            collision = link.find('collision')
            self.assertEqual(list(map(float, collision.find('origin').get('xyz').split())), [0.,0.,.01])
            self.assertEqual(list(map(float, collision.find('geometry/mesh').get('scale').split())), [1.2,1.,1.])
            spacer = link.find("collision[@name='finger_spacer']")
            self.assertEqual(list(map(float, spacer.find('geometry/box').get('size').split())), [.0252,.0264,.01])
        self.assertEqual(model.find('link/inertial/mass').get('value'), '0.014')
        self.assertEqual(model.find('joint/limit').get('upper'), '0.04')
        self.assertEqual(model.findall('link')[1].find('collision/origin').get('rpy'), '0 0 3.141592653589793')

    def test_invalid_geometry_is_rejected(self):
        with self.assertRaises(ValueError):
            apply_finger_spacer(self.model(), {'length_m': float('nan'), 'size_xy_m':[.021,.0264]})


if __name__ == '__main__':
    unittest.main()
