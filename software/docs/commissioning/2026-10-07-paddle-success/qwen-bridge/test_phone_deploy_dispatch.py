"""Offline dispatch tests: phone mode must never enter the other service paths."""
import argparse
import ast
import contextlib
import io
import json
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import Mock, patch

source = Path(__file__).with_name('redeploy_robot_server.py')
module = ast.parse(source.read_text())
entry = next(n for n in module.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
namespace = dict(argparse=argparse, Path=Path, sys=sys, json=json, ROOT=Path('/offline/robot'), __doc__='test')
exec(compile(ast.Module(body=[entry], type_ignores=[]), str(source), 'exec'), namespace)

class DispatchTests(unittest.TestCase):
    def run_mode(self, args, ok=True):
        helper = types.ModuleType('phone_camera_deploy')
        helper.deploy_phone_camera = Mock(return_value={'ok': ok, 'state':'dry_run'})
        with patch.dict(sys.modules, phone_camera_deploy=helper), patch.object(sys, 'argv', ['redeploy', *args]), contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            try:
                namespace['main']()
                code = 0
            except SystemExit as e:
                code = e.code
        return code, helper.deploy_phone_camera

    def test_dry_run_dispatches_only_phone_helper(self):
        code, helper = self.run_mode(['--phone-only','--dry-run'])
        self.assertEqual(code, 0)
        helper.assert_called_once_with(Path('/offline/robot'), dry_run=True)

    def test_phone_write_dispatches_only_phone_helper(self):
        code, helper = self.run_mode(['--phone-only'])
        self.assertEqual(code, 0)
        helper.assert_called_once_with(Path('/offline/robot'), dry_run=False)

    def test_failed_phone_deploy_returns_failure(self):
        code, helper = self.run_mode(['--phone-only'], ok=False)
        self.assertEqual(code, 1)
        self.assertEqual(helper.call_count, 1)

    def test_other_service_flags_rejected_before_phone_operation(self):
        for options in (['--api-only'], ['--network-only'], ['--cameras-only'], ['--oak','off'], ['--release-holding'], ['--no-head'], ['--no-wheels'], ['--no-wrist-cams'], ['--right-arm-only'], ['--joycon-teleop'], ['--native-joycon-reference'], ['--upstream-joycon-reference','/offline/x']):
            with self.subTest(options=options):
                code, helper = self.run_mode(['--phone-only', *options])
                self.assertEqual(code, 2)
                helper.assert_not_called()

if __name__ == '__main__': unittest.main()
