#!/usr/bin/env python3

import os
import shutil
import sys
import tempfile
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

import run_four_paper_ball_competition_preflight as preflight  # noqa: E402


WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


class FourPaperBallCompetitionPreflightTest(unittest.TestCase):
    def check_by_name(self, summary, name):
        matches = [item for item in summary['checks'] if item['name'] == name]
        self.assertEqual(1, len(matches))
        return matches[0]

    def test_current_workspace_passes_static_competition_preflight(self):
        summary = preflight.build_preflight_summary(WORKSPACE)

        self.assertEqual('COMPETITION_PREFLIGHT_PASSED', summary['status'])
        self.assertEqual(['bottle', 'paper_ball', 'box'], summary['pickup_classes'])
        self.assertEqual(['trash_bin'], summary['dropoff_classes'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'wp2_kitchen_waypoint_frozen')['status'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'wp6_final_exit_frozen')['status'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'competition_audit_present')['status'])
        self.assertTrue(any('senior' in item for item in summary['migration_checklist']))
        self.assertTrue(any('source' in item and 'devel/setup.bash' in item
                            for item in summary['migration_checklist']))

    def test_preflight_rejects_removed_wp6_exit(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            self.copy_minimal_workspace(temp_dir)
            flow_path = os.path.join(
                temp_dir, 'src', 'garbage_localization', 'scripts',
                'run_robocup_task_flow.py')
            with open(flow_path) as handle:
                flow = handle.read()
            with open(flow_path, 'w') as handle:
                handle.write(flow.replace('0.190553', '0.290553'))

            summary = preflight.build_preflight_summary(temp_dir)

        self.assertEqual('COMPETITION_PREFLIGHT_FAILED', summary['status'])
        self.assertEqual('FAIL', self.check_by_name(
            summary, 'wp6_final_exit_frozen')['status'])

    def test_preflight_rejects_raised_localization_threshold(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            self.copy_minimal_workspace(temp_dir)
            config_path = os.path.join(
                temp_dir, 'src', 'garbage_localization', 'config',
                'enhanced_sim.yaml')
            with open(config_path) as handle:
                config = handle.read()
            with open(config_path, 'w') as handle:
                handle.write(config.replace(
                    'minimum_detection_confidence: 0.0001',
                    'minimum_detection_confidence: 0.001'))

            summary = preflight.build_preflight_summary(temp_dir)

        self.assertEqual('COMPETITION_PREFLIGHT_FAILED', summary['status'])
        self.assertEqual('FAIL', self.check_by_name(
            summary, 'localization_keeps_trash_bin_but_low_threshold')['status'])

    def copy_minimal_workspace(self, temp_dir):
        files = [
            'src/garbage_localization/config/enhanced_sim.yaml',
            'src/garbage_localization/scripts/run_robocup_task_flow.py',
            'src/garbage_localization/scripts/run_four_paper_ball_fresh_regression.py',
            'src/garbage_localization/test/test_robocup_task_flow.py',
            'src/garbage_localization/test/test_sequential_navigation_regression.py',
            'src/nav_pkg/launch/nav.launch',
        ]
        for rel in files:
            source = os.path.join(WORKSPACE, rel)
            dest = os.path.join(temp_dir, rel)
            os.makedirs(os.path.dirname(dest), exist_ok=True)
            shutil.copy2(source, dest)
        setup = os.path.join(temp_dir, 'devel', 'setup.bash')
        os.makedirs(os.path.dirname(setup), exist_ok=True)
        with open(setup, 'w') as handle:
            handle.write('# test setup\n')


if __name__ == '__main__':
    unittest.main()
