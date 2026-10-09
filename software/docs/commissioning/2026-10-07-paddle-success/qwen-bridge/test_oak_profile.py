"""A healthy OAK and Joy-Con owner coexist across camera-only reconciliation."""
import json
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import patch

import redeploy_robot_server as D


class ProfileTests(unittest.TestCase):
    def test_only_fresh_reviewed_configuration_is_persisted_without_process_calls(self):
        for age, config, dry in ((.1, 'reviewed', False), (.1, 'reviewed', True),
                                 (5, 'reviewed', False), (.1, 'changed', False)):
            with self.subTest(age=age, config=config, dry=dry), tempfile.TemporaryDirectory() as tmp:
                root = Path(tmp)
                (root/'config').mkdir()
                (root/'config/oak-policy-camera-20261009.json').write_text(json.dumps(
                    {'source': {'config_sha256': 'reviewed'}}))
                (root/'oak.json').write_text(json.dumps(dict(config_sha256=config,
                    device_id='oak-unit', captured_at=time.time()-age)))
                with patch.multiple(D, SOFTWARE=root, OAK_RAW_DIR=str(root), OAK_OFF=root/'off'), \
                     patch.object(D, 'oak_processes', return_value=['123 python --wide']), \
                     patch.object(D, 'oak_fresh', return_value=True), \
                     patch.object(D.subprocess, 'Popen') as start, \
                     patch.object(D.os, 'killpg') as stop, patch.object(D, 'say'):
                    D.ensure_oak(dry)
                    start.assert_not_called()
                    stop.assert_not_called()
                saved = root/'oak-profile.json'
                self.assertEqual(saved.exists(), age < 1 and config == 'reviewed' and not dry)
                if saved.exists():
                    self.assertEqual(json.loads(saved.read_text())['fps'], 10)


if __name__ == '__main__':
    unittest.main()
