import unittest
from runtime_profile import local_control_arguments


class LocalControlTest(unittest.TestCase):
    def test_default_control_channel_is_loopback_and_expected_port(self):
        args=local_control_arguments({})
        self.assertIn('isaacsim.code_editor.python_server',args)
        self.assertIn('--/exts/isaacsim.code_editor.python_server/host=127.0.0.1',args)
        self.assertIn('--/exts/isaacsim.code_editor.python_server/port=8226',args)

    def test_explicit_extension_owner_is_not_duplicated(self):
        self.assertEqual(local_control_arguments({'mcp':{'port':8226}}),[])


if __name__ == '__main__':
    unittest.main()
