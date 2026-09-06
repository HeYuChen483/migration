#!/usr/bin/env python3

import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from run_sequential_navigation_regression import (  # noqa: E402
    ALL_CASES,
    DEFAULT_CASES,
    EXTENDED_CASES,
    assert_case_passed,
    run_case,
    selected_cases,
    task_flow_command,
)


class Args(object):
    visit_count = 4
    minimum_obstacle_distance = 0.55
    minimum_path_obstacle_distance = 0.30


class SequentialNavigationRegressionTest(unittest.TestCase):
    def test_assert_case_passed_accepts_complete_safe_summary(self):
        output = '''noise before
{
  "mode": "sequential_move_base_action",
  "navigation_attempts": 4,
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_COMPLETE",
  "targets": [
    {"approach": {"nearest_map_obstacle_distance_m": 0.558}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.551}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.627}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.557}}
  ],
  "visited_targets": 4
}
noise after
'''
        result = assert_case_passed(Args(), {'name': 'case'}, output)

        self.assertEqual('case', result['case'])
        self.assertEqual(4, result['visited_targets'])
        self.assertEqual(4, result['navigation_attempts'])

    def test_assert_case_passed_rejects_partial_summary(self):
        output = '''{
  "mode": "sequential_move_base_action",
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_PARTIAL",
  "targets": [],
  "visited_targets": 3
}'''

        with self.assertRaises(RuntimeError):
            assert_case_passed(Args(), {'name': 'case'}, output)

    def test_assert_case_passed_rejects_unsafe_target(self):
        output = '''{
  "mode": "sequential_move_base_action",
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_COMPLETE",
  "targets": [
    {"approach": {"nearest_map_obstacle_distance_m": 0.558}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.549}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.627}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.557}}
  ],
  "visited_targets": 4
}'''

        with self.assertRaises(RuntimeError):
            assert_case_passed(Args(), {'name': 'case'}, output)


    def test_assert_case_passed_accepts_path_above_path_threshold(self):
        output = '''{
  "mode": "sequential_move_base_action",
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_COMPLETE",
  "targets": [
    {"approach": {"nearest_map_obstacle_distance_m": 0.558,
                   "path_nearest_map_obstacle_distance_m": 0.350}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.620,
                   "path_nearest_map_obstacle_distance_m": 0.310}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.627,
                   "path_nearest_map_obstacle_distance_m": 0.300}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.557,
                   "path_nearest_map_obstacle_distance_m": 0.400}}
  ],
  "visited_targets": 4
}'''

        result = assert_case_passed(Args(), {'name': 'case'}, output)

        self.assertEqual('case', result['case'])


    def test_assert_case_passed_rejects_path_below_path_threshold(self):
        output = '''{
  "mode": "sequential_move_base_action",
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_COMPLETE",
  "targets": [
    {"approach": {"nearest_map_obstacle_distance_m": 0.558,
                   "path_nearest_map_obstacle_distance_m": 0.350}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.620,
                   "path_nearest_map_obstacle_distance_m": 0.299}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.627,
                   "path_nearest_map_obstacle_distance_m": 0.310}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.557,
                   "path_nearest_map_obstacle_distance_m": 0.400}}
  ],
  "visited_targets": 4
}'''

        with self.assertRaises(RuntimeError):
            assert_case_passed(Args(), {'name': 'case'}, output)

    def test_default_selection_runs_small_start_yaw_matrix(self):
        cases = selected_cases(None)

        self.assertEqual(4, len(cases))
        self.assertEqual([case['name'] for case in DEFAULT_CASES],
                         [case['name'] for case in cases])
        self.assertIn('four_occlusion_default_yaw0',
                      [case['name'] for case in cases])
        self.assertIn('four_occlusion_default_yaw90',
                      [case['name'] for case in cases])
        self.assertIn('four_occlusion_left_yaw0',
                      [case['name'] for case in cases])
        self.assertIn('four_occlusion_right_yaw0',
                      [case['name'] for case in cases])

    def test_extended_selection_runs_extra_start_yaw_matrix(self):
        cases = selected_cases(['extended'])

        self.assertEqual([case['name'] for case in EXTENDED_CASES],
                         [case['name'] for case in cases])
        self.assertIn('four_occlusion_inside_yaw90',
                      [case['name'] for case in cases])

    def test_all_full_selection_runs_default_and_extended_cases(self):
        cases = selected_cases(['all-full'])

        self.assertEqual([case['name'] for case in ALL_CASES],
                         [case['name'] for case in cases])

    def test_single_extended_case_can_be_selected(self):
        cases = selected_cases(['four_occlusion_inside_yaw0'])

        self.assertEqual(1, len(cases))
        self.assertEqual('four_occlusion_inside_yaw0', cases[0]['name'])

    def test_legacy_yaw0_case_alias_selects_default_yaw0(self):
        cases = selected_cases(['four_occlusion_yaw0'])

        self.assertEqual(1, len(cases))
        self.assertEqual('four_occlusion_default_yaw0', cases[0]['name'])

    def test_parse_args_accepts_extended_selectors(self):
        import run_sequential_navigation_regression
        args = run_sequential_navigation_regression.parse_args([
            '--case', 'extended',
            '--case', 'four_occlusion_inside_yaw0',
            '--case', 'all-full',
        ])

        self.assertEqual([
            'extended',
            'four_occlusion_inside_yaw0',
            'all-full',
        ], args.case)

    def test_task_flow_command_passes_min_confidence_filter(self):
        import run_sequential_navigation_regression
        args = run_sequential_navigation_regression.parse_args([])

        command = task_flow_command(args, DEFAULT_CASES[0])

        self.assertIn('--min-targets', command)
        min_targets_index = command.index('--min-targets')
        self.assertEqual('1', command[min_targets_index + 1])
        self.assertIn('--min-confidence', command)
        index = command.index('--min-confidence')
        self.assertEqual('0.001', command[index + 1])

    def test_run_case_accepts_complete_summary_after_timeout_cleanup(self):
        import tempfile
        import run_sequential_navigation_regression

        class Process(object):
            pid = 4321
            returncode = 124

            def __init__(self, *_args, **kwargs):
                kwargs['stdout'].write('''{
  "mode": "sequential_move_base_action",
  "navigation_attempts": 4,
  "requested_targets": 4,
  "status": "SEQUENTIAL_NAVIGATION_COMPLETE",
  "targets": [
    {"approach": {"nearest_map_obstacle_distance_m": 0.558,
                   "path_nearest_map_obstacle_distance_m": 0.350}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.551,
                   "path_nearest_map_obstacle_distance_m": 0.310}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.627,
                   "path_nearest_map_obstacle_distance_m": 0.300}},
    {"approach": {"nearest_map_obstacle_distance_m": 0.557,
                   "path_nearest_map_obstacle_distance_m": 0.400}}
  ],
  "visited_targets": 4
}
Aborted (core dumped)
''')

            def wait(self, timeout=None):
                return self.returncode

        args = type('Args', (), {
            'workspace': '/tmp/ws',
            'log_dir': tempfile.mkdtemp(),
            'command_timeout': 1.0,
            'visit_count': 4,
            'minimum_obstacle_distance': 0.55,
            'minimum_path_obstacle_distance': 0.30,
            'min_confidence': 0.001,
            'startup_wait': 0.0,
            'timeout': 0.0,
            'initial_scan_timeout': 0.0,
            'recovery_scan_timeout': 0.0,
            'navigation_timeout': 0.0,
        })()
        case = {'name': 'case', 'scenario': 'four_occlusion'}
        original_popen = run_sequential_navigation_regression.subprocess.Popen
        try:
            run_sequential_navigation_regression.subprocess.Popen = Process
            result = run_case(args, case)
        finally:
            run_sequential_navigation_regression.subprocess.Popen = original_popen

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', result['status'])
        self.assertEqual(4, result['visited_targets'])

    def test_run_case_times_out_and_cleans_process_group(self):
        import tempfile
        import run_sequential_navigation_regression

        class Process(object):
            pid = 4321
            returncode = None

            def __init__(self, *_args, **_kwargs):
                created['process'] = self
                self.wait_calls = []

            def wait(self, timeout=None):
                self.wait_calls.append(timeout)
                if timeout is not None:
                    raise run_sequential_navigation_regression.subprocess.TimeoutExpired(
                        'cmd', timeout)
                self.returncode = -15
                return self.returncode

            def poll(self):
                return self.returncode

        created = {}
        terminations = []
        original_popen = run_sequential_navigation_regression.subprocess.Popen
        original_killpg = run_sequential_navigation_regression.os.killpg
        original_sleep = run_sequential_navigation_regression.time.sleep
        try:
            run_sequential_navigation_regression.subprocess.Popen = Process
            run_sequential_navigation_regression.os.killpg = (
                lambda pid, sig: terminations.append((pid, sig)))
            run_sequential_navigation_regression.time.sleep = lambda _seconds: None
            args = type('Args', (), {
                'workspace': '/tmp/ws',
                'log_dir': tempfile.mkdtemp(),
                'command_timeout': 0.01,
                'visit_count': 4,
                'minimum_obstacle_distance': 0.55,
                'minimum_path_obstacle_distance': 0.30,
                'min_confidence': 0.001,
                'startup_wait': 0.0,
                'timeout': 0.0,
                'initial_scan_timeout': 0.0,
                'recovery_scan_timeout': 0.0,
                'navigation_timeout': 0.0,
            })()
            with self.assertRaises(RuntimeError) as context:
                run_case(args, {'name': 'timeout_case', 'scenario': 'four_occlusion'})
        finally:
            run_sequential_navigation_regression.subprocess.Popen = original_popen
            run_sequential_navigation_regression.os.killpg = original_killpg
            run_sequential_navigation_regression.time.sleep = original_sleep

        self.assertIn('timed out', str(context.exception))
        self.assertEqual(4321, terminations[0][0])
        self.assertEqual(run_sequential_navigation_regression.signal.SIGTERM,
                         terminations[0][1])
        self.assertEqual([0.01, None], created['process'].wait_calls)

    def test_default_timeouts_allow_slow_gazebo_perception(self):
        import run_sequential_navigation_regression
        args = run_sequential_navigation_regression.parse_args([])

        self.assertEqual(30.0, args.timeout)
        self.assertEqual(24.0, args.initial_scan_timeout)
        self.assertEqual(24.0, args.recovery_scan_timeout)
        self.assertEqual(720.0, args.command_timeout)
        self.assertEqual(0.001, args.min_confidence)


class FourPaperBallFreshRegressionTest(unittest.TestCase):
    def test_assert_four_paper_ball_passed_accepts_validated_full_flow(self):
        import run_four_paper_ball_fresh_regression as fresh
        args = fresh.parse_args([])
        output = '''noise before
{
  "mode": "four_room_waypoint_patrol",
  "status": "ROOM_PATROL_COMPLETE",
  "visited_targets": 4,
  "completed_rooms": ["living_room", "kitchen", "bedroom", "dining_room"],
  "final_exit": {"status": "EXIT_REACHED"},
  "targets": [
    {"room": "living_room", "target": {"class_name": "bottle", "confidence": 0.21},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "kitchen", "target": {"class_name": "bottle", "confidence": 0.0013},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "bedroom", "target": {"class_name": "paper_ball", "confidence": 0.0012},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "dining_room", "target": {"class_name": "box", "confidence": 0.0037},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}}
  ]
}
noise after
'''

        result = fresh.assert_four_paper_ball_passed(args, output)

        self.assertEqual('ROOM_PATROL_COMPLETE', result['status'])
        self.assertEqual(4, result['visited_targets'])
        self.assertEqual('EXIT_REACHED', result['final_exit'])
        self.assertEqual('bottle', result['targets'][1]['class_name'])
        self.assertAlmostEqual(0.0013, result['targets'][1]['confidence'])

    def test_enhanced_sim_localization_threshold_matches_weak_kitchen_bottle(self):
        config_path = os.path.join(
            os.path.dirname(__file__), '..', 'config', 'enhanced_sim.yaml')
        with open(config_path) as handle:
            config = handle.read()

        self.assertIn('minimum_detection_confidence: 0.0001', config)

    def test_assert_four_paper_ball_passed_rejects_missing_dropoff(self):
        import run_four_paper_ball_fresh_regression as fresh
        args = fresh.parse_args([])
        output = '''{
  "mode": "four_room_waypoint_patrol",
  "status": "ROOM_PATROL_COMPLETE",
  "visited_targets": 4,
  "completed_rooms": ["living_room", "kitchen", "bedroom", "dining_room"],
  "final_exit": {"status": "EXIT_REACHED"},
  "targets": [
    {"room": "living_room", "target": {"class_name": "bottle"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "kitchen", "target": {"class_name": "bottle"}, "status": "TARGET_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "bedroom", "target": {"class_name": "paper_ball"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "dining_room", "target": {"class_name": "box"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}}
  ]
}'''

        with self.assertRaises(RuntimeError):
            fresh.assert_four_paper_ball_passed(args, output)


class ThreeGarbageCompetitionRegressionTest(unittest.TestCase):
    def test_task_flow_command_enables_opt_in_competition_mode(self):
        import run_three_garbage_competition_regression as competition
        args = competition.parse_args([])

        command = competition.task_flow_command(args)

        self.assertIn('--garbage-competition-mode', command)
        self.assertIn('--garbage-quota', command)
        quota_index = command.index('--garbage-quota')
        self.assertEqual('3', command[quota_index + 1])
        self.assertIn('--room-patrol', command)
        patrol_index = command.index('--room-patrol')
        self.assertEqual('on', command[patrol_index + 1])
        self.assertIn('--send-goal', command)
        self.assertIn('--no-launch-stack', command)
        self.assertNotIn('--visit-count', command)

    def test_assert_three_garbage_competition_passed_accepts_quota_flow(self):
        import run_three_garbage_competition_regression as competition
        args = competition.parse_args([])
        output = '''noise before
{
  "mode": "four_room_waypoint_patrol",
  "status": "ROOM_PATROL_COMPLETE",
  "garbage_competition_mode": true,
  "garbage_quota": 3,
  "requested_targets": 3,
  "visited_targets": 3,
  "completed_dropoffs": 3,
  "completed_rooms": ["living_room", "kitchen", "bedroom"],
  "skipped_rooms_after_quota": [
    {"room": "dining_room", "waypoint": "wp5_dining_room_360_scan",
     "status": "SKIPPED_GARBAGE_QUOTA_REACHED", "reason": "garbage_quota_reached", "garbage_quota": 3}
  ],
  "final_exit": {"status": "EXIT_REACHED"},
  "competition_audit": {"status": "COMPETITION_AUDIT_PASSED"},
  "targets": [
    {"room": "living_room", "target": {"class_name": "bottle", "confidence": 0.21},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "kitchen", "target": {"class_name": "bottle", "confidence": 0.0013},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "bedroom", "target": {"class_name": "paper_ball", "confidence": 0.0012},
     "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}}
  ]
}
noise after
'''

        result = competition.assert_three_garbage_competition_passed(args, output)

        self.assertEqual('ROOM_PATROL_COMPLETE', result['status'])
        self.assertTrue(result['garbage_competition_mode'])
        self.assertEqual(3, result['requested_targets'])
        self.assertEqual(3, result['visited_targets'])
        self.assertEqual(3, result['completed_dropoffs'])
        self.assertEqual(['living_room', 'kitchen', 'bedroom'], result['completed_rooms'])
        self.assertEqual('dining_room', result['skipped_rooms_after_quota'][0]['room'])
        self.assertEqual('EXIT_REACHED', result['final_exit'])
        self.assertEqual('COMPETITION_AUDIT_PASSED', result['competition_audit'])
        self.assertEqual(['bottle', 'bottle', 'paper_ball'],
                         [target['class_name'] for target in result['targets']])

    def test_assert_three_garbage_competition_passed_rejects_fourth_room_target(self):
        import run_three_garbage_competition_regression as competition
        args = competition.parse_args([])
        output = '''{
  "mode": "four_room_waypoint_patrol",
  "status": "ROOM_PATROL_COMPLETE",
  "garbage_competition_mode": true,
  "requested_targets": 3,
  "visited_targets": 3,
  "completed_dropoffs": 3,
  "completed_rooms": ["living_room", "kitchen", "bedroom"],
  "skipped_rooms_after_quota": [
    {"room": "dining_room", "status": "SKIPPED_GARBAGE_QUOTA_REACHED"}
  ],
  "final_exit": {"status": "EXIT_REACHED"},
  "competition_audit": {"status": "COMPETITION_AUDIT_PASSED"},
  "targets": [
    {"room": "living_room", "target": {"class_name": "bottle"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "kitchen", "target": {"class_name": "bottle"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}},
    {"room": "dining_room", "target": {"class_name": "box"}, "status": "DROPOFF_REACHED",
     "pre_pickup_dropoff": {"status": "DROPOFF_CONFIRMED", "source": "current_perception"}}
  ]
}'''

        with self.assertRaises(RuntimeError):
            competition.assert_three_garbage_competition_passed(args, output)


if __name__ == '__main__':
    unittest.main()
