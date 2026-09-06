#!/usr/bin/env python3
"""Offline competition/transfer preflight for the four_paper_ball task.

This checker intentionally does not launch ROS, Gazebo, RViz, or move_base.  It
only verifies the frozen task-layer files before the package is copied to another
computer or used for a real-robot competition rehearsal.
"""

from __future__ import print_function

import argparse
import json
import os
import re
import sys


EXPECTED_WP2 = {
    'name': 'wp2_kitchen_260_scan',
    'x': '7.02370',
    'y': '6.98012',
    'yaw': '1.5059047125371625',
    'scan_offsets': ('65.0', '130.0', '195.0', '260.0'),
}
EXPECTED_WP6 = {
    'name': 'wp6_left_dining_room_exit',
    'x': '0.190553',
    'y': '7.89226',
    'yaw': '-3.084572395452059',
}
PICKUP_CLASSES = ['bottle', 'paper_ball', 'box']
DROPOFF_CLASSES = ['trash_bin']
REQUIRED_FILES = [
    'src/garbage_localization/config/enhanced_sim.yaml',
    'src/garbage_localization/scripts/run_robocup_task_flow.py',
    'src/garbage_localization/scripts/run_four_paper_ball_fresh_regression.py',
    'src/garbage_localization/test/test_robocup_task_flow.py',
    'src/garbage_localization/test/test_sequential_navigation_regression.py',
    'src/nav_pkg/launch/nav.launch',
]


class PreflightError(RuntimeError):
    pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=('Run offline competition/transfer checks for the '
                     'four_paper_ball RoboCup task.'))
    parser.add_argument('--workspace', default='/home/hyc/catkin_wa',
                        help='Catkin workspace root. Default: /home/hyc/catkin_wa')
    parser.add_argument('--json-only', action='store_true',
                        help='Print only the machine-readable JSON summary.')
    return parser.parse_args(argv)


def read_text(path):
    with open(path, 'r') as handle:
        return handle.read()


def add_check(checks, name, passed, details=None, required=True):
    if passed:
        status = 'OK'
    elif required:
        status = 'FAIL'
    else:
        status = 'WARN'
    item = {'name': name, 'status': status}
    if details is not None:
        item['details'] = details
    checks.append(item)
    return item


def relative_path(workspace, *parts):
    return os.path.join(workspace, *parts)


def find_wp_block(flow_text, waypoint_name):
    index = flow_text.find("'name': '{}'".format(waypoint_name))
    if index < 0:
        index = flow_text.find('"name": "{}"'.format(waypoint_name))
    if index < 0:
        return ''
    start = max(0, flow_text.rfind('{', 0, index))
    end = flow_text.find('}', index)
    if end < 0:
        end = index + 500
    return flow_text[start:end + 1]


def flow_has_wp2_expected(flow_text):
    block = find_wp_block(flow_text, EXPECTED_WP2['name'])
    return (EXPECTED_WP2['x'] in block and
            EXPECTED_WP2['y'] in block and
            EXPECTED_WP2['yaw'] in block and
            all(value in block for value in EXPECTED_WP2['scan_offsets']) and
            "'classes': ['bottle']" in block)


def flow_has_wp6_expected(flow_text):
    block = find_wp_block(flow_text, EXPECTED_WP6['name'])
    return (EXPECTED_WP6['x'] in block and
            EXPECTED_WP6['y'] in block and
            EXPECTED_WP6['yaw'] in block and
            'bottom_living_room_entrance' in block)


def flow_has_competition_audit(flow_text):
    return ('def build_competition_flow_audit' in flow_text and
            "competition_audit" in flow_text and
            'trash_bin_confirmed_before_each_pickup' in flow_text and
            'dropoff_reached_after_each_pickup' in flow_text)


def flow_has_hardware_safe_hooks(flow_text):
    return ('def run_pickup_action' in flow_text and
            'def run_dropoff_action' in flow_text and
            "hardware_commanded': False" in flow_text and
            "--pickup-action" in flow_text and
            "--dropoff-action" in flow_text)


def fresh_command_uses_competition_classes(fresh_text):
    return ("--target-classes', args.target_classes" in fresh_text and
            "--dropoff-classes', args.dropoff_classes" in fresh_text and
            "default='bottle,paper_ball,box'" in fresh_text and
            "default='trash_bin'" in fresh_text)


def config_has_required_detector_classes(config_text):
    compact = re.sub(r'\s+', '', config_text)
    return ('target_classes:[bottle,paper_ball,box,trash_bin]' in compact and
            'minimum_detection_confidence:0.0001' in compact)


def migration_checklist(workspace):
    return [
        'Copy the full catkin workspace or at least src/garbage_localization plus required src/nav_pkg dependencies to the senior\'s computer.',
        'On the senior\'s computer, build the workspace and run: source {}/devel/setup.bash'.format(workspace),
        'Before connecting hardware, run: python3 src/garbage_localization/scripts/run_four_paper_ball_competition_preflight.py --workspace {}'.format(workspace),
        'Run unit checks first: python3 -m unittest src/garbage_localization/test/test_robocup_task_flow.py src/garbage_localization/test/test_sequential_navigation_regression.py',
        'Only after ROS topics, frames, map path, camera/LiDAR drivers, and move_base action name match, run the Gazebo/RViz fresh regression or real-robot rehearsal.',
        'Keep pickup/dropoff action hooks at dry-run/none until the real actuator interface is explicitly connected and tested.',
    ]


def build_preflight_summary(workspace):
    workspace = os.path.abspath(workspace)
    checks = []
    add_check(checks, 'workspace_exists', os.path.isdir(workspace),
              {'workspace': workspace})

    missing = [path for path in REQUIRED_FILES
               if not os.path.exists(relative_path(workspace, *path.split('/')))]
    add_check(checks, 'required_files_present', not missing,
              {'missing': missing, 'required_files': REQUIRED_FILES})

    flow_path = relative_path(
        workspace, 'src', 'garbage_localization', 'scripts',
        'run_robocup_task_flow.py')
    fresh_path = relative_path(
        workspace, 'src', 'garbage_localization', 'scripts',
        'run_four_paper_ball_fresh_regression.py')
    config_path = relative_path(
        workspace, 'src', 'garbage_localization', 'config', 'enhanced_sim.yaml')
    setup_path = relative_path(workspace, 'devel', 'setup.bash')

    flow_text = read_text(flow_path) if os.path.exists(flow_path) else ''
    fresh_text = read_text(fresh_path) if os.path.exists(fresh_path) else ''
    config_text = read_text(config_path) if os.path.exists(config_path) else ''

    add_check(checks, 'pickup_dropoff_class_semantics',
              fresh_command_uses_competition_classes(fresh_text),
              {'pickup_classes': PICKUP_CLASSES,
               'dropoff_classes': DROPOFF_CLASSES,
               'trash_bin_pickup_allowed': False})
    add_check(checks, 'localization_keeps_trash_bin_but_low_threshold',
              config_has_required_detector_classes(config_text),
              {'expected_target_classes': PICKUP_CLASSES + DROPOFF_CLASSES,
               'expected_minimum_detection_confidence': 0.0001})
    add_check(checks, 'wp1_to_wp5_route_frozen',
              all(name in flow_text for name in [
                  'wp1_living_room_left_180_scan',
                  EXPECTED_WP2['name'],
                  'wp3_bedroom_front_check',
                  'wp4_bedroom_front_backup',
                  'wp5_dining_room_360_scan']) and
              'WP2B' not in flow_text and 'wp2b' not in flow_text,
              {'wp2b_allowed': False})
    add_check(checks, 'wp2_kitchen_waypoint_frozen', flow_has_wp2_expected(flow_text),
              EXPECTED_WP2)
    add_check(checks, 'wp6_final_exit_frozen', flow_has_wp6_expected(flow_text),
              EXPECTED_WP6)
    add_check(checks, 'competition_audit_present',
              flow_has_competition_audit(flow_text),
              {'checks': ['trash_bin_confirmed_before_each_pickup',
                          'dropoff_reached_after_each_pickup',
                          'final_exit_recorded_after_completed_dropoffs']})
    add_check(checks, 'dry_run_hooks_hardware_safe',
              flow_has_hardware_safe_hooks(flow_text),
              {'default_safe_modes': ['dry-run', 'none']})
    add_check(checks, 'catkin_setup_available_for_transfer',
              os.path.exists(setup_path),
              {'setup': setup_path}, required=False)

    status = ('COMPETITION_PREFLIGHT_PASSED'
              if all(item['status'] != 'FAIL' for item in checks)
              else 'COMPETITION_PREFLIGHT_FAILED')
    return {
        'status': status,
        'workspace': workspace,
        'scenario': 'four_paper_ball',
        'pickup_classes': PICKUP_CLASSES,
        'dropoff_classes': DROPOFF_CLASSES,
        'checks': checks,
        'migration_checklist': migration_checklist(workspace),
    }


def print_human_summary(summary):
    print(summary['status'])
    print('workspace={}'.format(summary['workspace']))
    for item in summary['checks']:
        print('[{}] {}'.format(item['status'], item['name']))
    print('\nMigration checklist for senior computer:')
    for index, item in enumerate(summary['migration_checklist'], 1):
        print('{}. {}'.format(index, item))


def main(argv=None):
    args = parse_args(argv)
    summary = build_preflight_summary(args.workspace)
    if not args.json_only:
        print_human_summary(summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'COMPETITION_PREFLIGHT_PASSED' else 1


if __name__ == '__main__':
    sys.exit(main())
