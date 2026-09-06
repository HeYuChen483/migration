#!/usr/bin/env python3
"""Real-robot transfer precheck for four_paper_ball.

This script does not launch the task flow, Gazebo, RViz, or any hardware action.
It checks that the senior computer / real robot runtime exposes the ROS topics,
TF endpoints, map file, and move_base action surface expected by the task.
"""

from __future__ import print_function

import argparse
import json
import os
import re
import subprocess
import sys
import time


DEFAULT_PROFILE = 'src/garbage_localization/config/four_paper_ball_real_robot_profile.template.yaml'
REQUIRED_PICKUP_CLASSES = ['bottle', 'paper_ball', 'box']
REQUIRED_DROPOFF_CLASSES = ['trash_bin']


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Check real-robot ROS environment before four_paper_ball task runs.')
    parser.add_argument('--workspace', default='/home/hyc/catkin_wa',
                        help='Catkin workspace root. Default: /home/hyc/catkin_wa')
    parser.add_argument('--profile', default=None,
                        help='Profile yaml path. Default: real-robot template in the workspace.')
    parser.add_argument('--live-ros', action='store_true',
                        help='Check a currently running ROS master/topics/TF/move_base.')
    parser.add_argument('--json-only', action='store_true',
                        help='Print only JSON output.')
    return parser.parse_args(argv)


def strip_comment(line):
    return line.split('#', 1)[0].strip()


def parse_scalar(value):
    value = value.strip()
    if value.startswith('[') and value.endswith(']'):
        body = value[1:-1].strip()
        if not body:
            return []
        return [item.strip().strip('"\'') for item in body.split(',')]
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    try:
        if re.match(r'^[-+]?\d+$', value):
            return int(value)
        if re.match(r'^[-+]?\d+(?:\.\d+)?$', value):
            return float(value)
    except ValueError:
        pass
    return value.strip('"\'')


def read_simple_yaml(path):
    data = {}
    stack = [(-1, data)]
    with open(path) as handle:
        for raw_line in handle:
            if not strip_comment(raw_line):
                continue
            indent = len(raw_line) - len(raw_line.lstrip(' '))
            line = strip_comment(raw_line)
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            while stack and indent <= stack[-1][0]:
                stack.pop()
            parent = stack[-1][1]
            if value:
                parent[key] = parse_scalar(value)
            else:
                child = {}
                parent[key] = child
                stack.append((indent, child))
    return data


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


def source_command(workspace, command):
    setup = os.path.join(workspace, 'devel', 'setup.bash')
    return 'source "{}" >/dev/null 2>&1 && {}'.format(setup, command)


def run_sourced(workspace, command, timeout):
    try:
        output = subprocess.check_output(
            ['bash', '-lc', source_command(workspace, command)],
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            timeout=timeout)
        return True, output
    except subprocess.CalledProcessError as error:
        return False, error.output
    except subprocess.TimeoutExpired as error:
        output = error.output or ''
        return False, 'timeout after {:.1f}s {}'.format(timeout, output)
    except OSError as error:
        return False, str(error)


def topic_available(topic, topic_list):
    return topic in topic_list.splitlines()


def move_base_topics(action_name):
    base = action_name.rstrip('/') or '/move_base'
    if not base.startswith('/'):
        base = '/' + base
    return [base + suffix for suffix in ['/status', '/goal', '/result']]


def live_ros_checks(workspace, profile):
    checks = []
    timeout = float(profile.get('precheck_timeouts', {}).get('ros_master_sec', 5.0))
    ok, node_output = run_sourced(workspace, 'rosnode list', timeout)
    add_check(checks, 'ros_master_reachable', ok,
              {'command': 'rosnode list', 'output_excerpt': node_output[:500]})
    if not ok:
        return checks

    ok, topics_output = run_sourced(workspace, 'rostopic list',
                                    float(profile.get('precheck_timeouts', {}).get('topic_sec', 2.0)))
    add_check(checks, 'rostopic_list_available', ok,
              {'command': 'rostopic list', 'output_excerpt': topics_output[:500]})
    if not ok:
        return checks

    required_topics = [
        profile.get('object_topic'),
        profile.get('detection_topic'),
        profile.get('camera_info_topic'),
        profile.get('point_cloud_topic'),
        profile.get('scan_topic'),
        profile.get('tf_topic', '/tf'),
        profile.get('tf_static_topic', '/tf_static'),
    ]
    missing_topics = [topic for topic in required_topics
                      if topic and not topic_available(topic, topics_output)]
    add_check(checks, 'required_ros_topics_present', not missing_topics,
              {'missing_topics': missing_topics,
               'required_topics': [topic for topic in required_topics if topic]})

    missing_action_topics = [topic for topic in move_base_topics(
        profile.get('move_base_action', '/move_base'))
        if not topic_available(topic, topics_output)]
    add_check(checks, 'move_base_action_topics_present', not missing_action_topics,
              {'missing_topics': missing_action_topics,
               'required_topics': move_base_topics(profile.get('move_base_action', '/move_base'))})

    frame_a = profile.get('navigation_frame', 'map')
    frame_b = profile.get('robot_base_frame', 'base_footprint')
    tf_timeout = float(profile.get('precheck_timeouts', {}).get('tf_sec', 3.0))
    command = 'timeout {:.1f}s rosrun tf tf_echo {} {}'.format(tf_timeout, frame_a, frame_b)
    ok, tf_output = run_sourced(workspace, command, tf_timeout + 1.0)
    add_check(checks, 'tf_navigation_to_robot_available', ok,
              {'command': command, 'from': frame_a, 'to': frame_b,
               'output_excerpt': tf_output[:500]})
    return checks


def build_precheck_summary(workspace, profile_path=None, live_ros=False):
    workspace = os.path.abspath(workspace)
    if profile_path is None:
        profile_path = os.path.join(workspace, DEFAULT_PROFILE)
    elif not os.path.isabs(profile_path):
        profile_path = os.path.join(workspace, profile_path)

    checks = []
    add_check(checks, 'workspace_exists', os.path.isdir(workspace),
              {'workspace': workspace})
    add_check(checks, 'catkin_setup_exists',
              os.path.exists(os.path.join(workspace, 'devel', 'setup.bash')),
              {'setup': os.path.join(workspace, 'devel', 'setup.bash')},
              required=live_ros)
    add_check(checks, 'profile_exists', os.path.exists(profile_path),
              {'profile': profile_path})

    profile = read_simple_yaml(profile_path) if os.path.exists(profile_path) else {}
    add_check(checks, 'profile_class_semantics',
              profile.get('target_classes') == REQUIRED_PICKUP_CLASSES and
              profile.get('dropoff_classes') == REQUIRED_DROPOFF_CLASSES,
              {'target_classes': profile.get('target_classes'),
               'dropoff_classes': profile.get('dropoff_classes'),
               'trash_bin_pickup_allowed': False})
    add_check(checks, 'profile_actions_remain_safe',
              profile.get('pickup_action') in ('dry-run', 'none') and
              profile.get('dropoff_action') in ('dry-run', 'none'),
              {'pickup_action': profile.get('pickup_action'),
               'dropoff_action': profile.get('dropoff_action')})
    rules = profile.get('real_robot_rules', {})
    add_check(checks, 'profile_real_robot_rules_enabled',
              bool(rules.get('keep_trash_bin_dropoff_only')) and
              bool(rules.get('keep_pickup_dropoff_actions_safe_until_hardware_tested')) and
              bool(rules.get('do_not_use_gazebo_truth_interfaces')),
              {'real_robot_rules': rules})

    map_path = profile.get('navigation_map')
    if map_path and not os.path.isabs(map_path):
        map_path = os.path.join(workspace, map_path)
    add_check(checks, 'navigation_map_path_exists',
              bool(map_path and os.path.exists(map_path)),
              {'navigation_map': map_path}, required=live_ros)

    if live_ros:
        checks.extend(live_ros_checks(workspace, profile))
    else:
        add_check(checks, 'live_ros_checks_skipped', True,
                  {'reason': 'Run with --live-ros after the senior computer or robot ROS stack is started.'},
                  required=False)

    status = ('REAL_ROBOT_PRECHECK_PASSED'
              if all(item['status'] != 'FAIL' for item in checks)
              else 'REAL_ROBOT_PRECHECK_FAILED')
    return {
        'status': status,
        'workspace': workspace,
        'profile': profile_path,
        'live_ros': live_ros,
        'checks': checks,
        'safe_task_command_template': [
            'rosrun', 'garbage_localization', 'run_robocup_task_flow.py',
            '--scenario', 'four_paper_ball',
            '--no-launch-stack',
            '--send-goal',
            '--visit-count', '4',
            '--room-patrol', 'on',
            '--target-classes', 'bottle,paper_ball,box',
            '--dropoff-classes', 'trash_bin',
            '--pickup-action', 'dry-run',
            '--dropoff-action', 'dry-run',
        ],
    }


def print_human_summary(summary):
    print(summary['status'])
    print('workspace={}'.format(summary['workspace']))
    print('profile={}'.format(summary['profile']))
    for item in summary['checks']:
        print('[{}] {}'.format(item['status'], item['name']))
    print('\nSafe task command template:')
    print(' '.join(summary['safe_task_command_template']))


def main(argv=None):
    args = parse_args(argv)
    summary = build_precheck_summary(args.workspace, args.profile, args.live_ros)
    if not args.json_only:
        print_human_summary(summary)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'REAL_ROBOT_PRECHECK_PASSED' else 1


if __name__ == '__main__':
    sys.exit(main())
