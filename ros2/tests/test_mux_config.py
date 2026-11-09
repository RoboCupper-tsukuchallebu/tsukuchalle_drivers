#!/usr/bin/env python3
"""Check resolved compose and mux parameters on the build machine."""
import json
import os
import subprocess
import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]


class MuxConfigTest(unittest.TestCase):
    def test_contract(self):
        config = yaml.safe_load((ROOT / 'ros2/config/twist_mux.yaml').read_text())
        params = config['twist_mux']['ros__parameters']
        self.assertIs(params['use_stamped'], False)
        topics = params['topics']
        self.assertEqual(topics['nav']['topic'], '/cmd_vel')
        self.assertEqual(topics['teleop']['topic'], '/cmd_vel_teleop')
        self.assertGreater(topics['teleop']['priority'], topics['nav']['priority'])
        for topic in topics.values():
            self.assertGreater(topic['timeout'], 0)
        env = dict(os.environ, DDT_SERIAL_DEVICE='/dev/test')
        result = subprocess.check_output(['docker', 'compose', '-f', str(ROOT / 'ros2/compose.yaml'),
                                          '--profile', 'keyboard', 'config', '--format', 'json'], env=env)
        services = json.loads(result)['services']
        for service, remap in [('mux', 'cmd_vel_out:=/cmd_vel_mux'),
                               ('drive', 'cmd_vel:=/cmd_vel_mux'),
                               ('teleop', 'cmd_vel:=/cmd_vel_teleop'),
                               ('teleop-keyboard', 'cmd_vel:=/cmd_vel_teleop')]:
            self.assertIn(remap, services[service]['command'])
        image = os.environ.get('DDT_TEST_IMAGE', 'ddt-can-bridge-ros2:mux')
        subprocess.run(['docker', 'run', '--rm', '--network', 'none', '--ipc', 'private',
                        '--entrypoint', 'test', image, '-x',
                        '/opt/ros/jazzy/lib/twist_mux/twist_mux'], check=True)


if __name__ == '__main__':
    unittest.main()
