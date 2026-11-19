"""Pure naming and confirmation contracts; no ROS node or model is started."""
import unittest
from ptc_inference.observe_stream import resolve_topic, station_topics, confirm_pick

class ObserveContractTest(unittest.TestCase):
    def test_endpoint_names_follow_the_station_profile(self):
        keys=('tracks','frame','vision_status','inference_status','detections','task_status',
              'task_command','gripper_status','arm_status','joint_states','rgb')
        profile={'runtime':{'namespace':'cell/right'},'ros':{'endpoints':{key:'custom/'+key for key in keys}}}
        topics=station_topics(profile)
        self.assertEqual(topics['task_status'],'/cell/right/custom/task_status')
        self.assertEqual(topics['bin_load'],'/cell/right/bin/load')
        with self.assertRaises(ValueError):resolve_topic('cell/right','/absolute')

    def test_confirmation_bridges_one_miss_but_requires_a_qualified_frame(self):
        streak,confirmed=confirm_pick(None,0,400_000_000)
        self.assertFalse(confirmed)
        streak,confirmed=confirm_pick(streak,200_000_000,400_000_000,False)
        self.assertFalse(confirmed)
        streak,confirmed=confirm_pick(streak,400_000_000,400_000_000,True)
        self.assertTrue(confirmed)
        streak,confirmed=confirm_pick(streak,600_000_000,400_000_000,False)
        self.assertFalse(confirmed)
        streak,confirmed=confirm_pick(streak,800_000_000,400_000_000,False)
        self.assertIsNone(streak)
        self.assertFalse(confirmed)

if __name__=='__main__':unittest.main()
