#!/usr/bin/env python3

import os
import shutil
import sys
import tempfile
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

import run_four_paper_ball_real_robot_precheck as precheck  # noqa: E402


WORKSPACE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))


class FourPaperBallRealRobotPrecheckTest(unittest.TestCase):
    def check_by_name(self, summary, name):
        matches = [item for item in summary['checks'] if item['name'] == name]
        self.assertEqual(1, len(matches))
        return matches[0]

    def test_current_template_passes_offline_precheck(self):
        summary = precheck.build_precheck_summary(WORKSPACE)

        self.assertEqual('REAL_ROBOT_PRECHECK_PASSED', summary['status'])
        self.assertFalse(summary['live_ros'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'profile_class_semantics')['status'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'profile_actions_remain_safe')['status'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'profile_real_robot_rules_enabled')['status'])
        self.assertEqual('OK', self.check_by_name(
            summary, 'live_ros_checks_skipped')['status'])
        command = summary['safe_task_command_template']
        self.assertIn('--pickup-action', command)
        self.assertEqual('dry-run', command[command.index('--pickup-action') + 1])
        self.assertEqual('bottle,paper_ball,box',
                         command[command.index('--target-classes') + 1])
        self.assertEqual('trash_bin', command[command.index('--dropoff-classes') + 1])

    def test_precheck_rejects_trash_bin_as_pickup_target(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = self.copy_profile_workspace(temp_dir)
            with open(profile_path) as handle:
                profile = handle.read()
            with open(profile_path, 'w') as handle:
                handle.write(profile.replace(
                    'target_classes: [bottle, paper_ball, box]',
                    'target_classes: [bottle, paper_ball, box, trash_bin]'))

            summary = precheck.build_precheck_summary(temp_dir)

        self.assertEqual('REAL_ROBOT_PRECHECK_FAILED', summary['status'])
        self.assertEqual('FAIL', self.check_by_name(
            summary, 'profile_class_semantics')['status'])

    def test_precheck_rejects_unsafe_hardware_actions(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = self.copy_profile_workspace(temp_dir)
            with open(profile_path) as handle:
                profile = handle.read()
            with open(profile_path, 'w') as handle:
                handle.write(profile.replace('pickup_action: dry-run',
                                             'pickup_action: hardware'))

            summary = precheck.build_precheck_summary(temp_dir)

        self.assertEqual('REAL_ROBOT_PRECHECK_FAILED', summary['status'])
        self.assertEqual('FAIL', self.check_by_name(
            summary, 'profile_actions_remain_safe')['status'])

    def test_missing_map_is_warning_offline_but_failure_for_live_ros(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            profile_path = self.copy_profile_workspace(temp_dir, copy_map=False)
            original_live_ros_checks = precheck.live_ros_checks
            try:
                precheck.live_ros_checks = lambda _workspace, _profile: []
                offline_summary = precheck.build_precheck_summary(temp_dir, profile_path)
                live_summary = precheck.build_precheck_summary(
                    temp_dir, profile_path, live_ros=True)
            finally:
                precheck.live_ros_checks = original_live_ros_checks

        self.assertEqual('REAL_ROBOT_PRECHECK_PASSED', offline_summary['status'])
        self.assertEqual('WARN', self.check_by_name(
            offline_summary, 'navigation_map_path_exists')['status'])
        self.assertEqual('REAL_ROBOT_PRECHECK_FAILED', live_summary['status'])
        self.assertEqual('FAIL', self.check_by_name(
            live_summary, 'navigation_map_path_exists')['status'])

    def test_move_base_topics_normalizes_action_name(self):
        self.assertEqual(['/move_base/status', '/move_base/goal', '/move_base/result'],
                         precheck.move_base_topics('move_base'))
        self.assertEqual(['/robot/move_base/status', '/robot/move_base/goal',
                          '/robot/move_base/result'],
                         precheck.move_base_topics('/robot/move_base/'))

    def copy_profile_workspace(self, temp_dir, copy_map=True):
        rel_profile = os.path.join(
            'src', 'garbage_localization', 'config',
            'four_paper_ball_real_robot_profile.template.yaml')
        source_profile = os.path.join(WORKSPACE, rel_profile)
        dest_profile = os.path.join(temp_dir, rel_profile)
        os.makedirs(os.path.dirname(dest_profile), exist_ok=True)
        shutil.copy2(source_profile, dest_profile)

        setup = os.path.join(temp_dir, 'devel', 'setup.bash')
        os.makedirs(os.path.dirname(setup), exist_ok=True)
        with open(setup, 'w') as handle:
            handle.write('# test setup\n')

        if copy_map:
            rel_map = os.path.join('src', 'wpr_simulation', 'maps', 'map.yaml')
            source_map = os.path.join(WORKSPACE, rel_map)
            dest_map = os.path.join(temp_dir, rel_map)
            os.makedirs(os.path.dirname(dest_map), exist_ok=True)
            shutil.copy2(source_map, dest_map)
            with open(dest_profile) as handle:
                profile = handle.read()
            with open(dest_profile, 'w') as handle:
                handle.write(profile.replace(
                    '/home/hyc/catkin_wa/src/wpr_simulation/maps/map.yaml',
                    rel_map))
        else:
            with open(dest_profile) as handle:
                profile = handle.read()
            with open(dest_profile, 'w') as handle:
                handle.write(profile.replace(
                    '/home/hyc/catkin_wa/src/wpr_simulation/maps/map.yaml',
                    'src/wpr_simulation/maps/missing_map.yaml'))
        return dest_profile


if __name__ == '__main__':
    unittest.main()
