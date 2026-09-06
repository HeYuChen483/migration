#!/usr/bin/env python3
"""Reproducible randomized three-garbage competition regression."""
from __future__ import print_function

import argparse
import json
import os
import random
import sys
import time

import run_three_garbage_competition_regression as fixed


SCENARIO = 'four_paper_ball'
SLOTS = {
    'living_room': ('enable_primary_garbage', 'bottle'),
    'kitchen': ('enable_second_garbage', 'bottle'),
    'bedroom': ('enable_paper_ball', 'paper_ball'),
    'dining_room': ('enable_box', 'box'),
}
PICKUP_CLASSES = set(['bottle', 'paper_ball', 'box'])


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run reproducible randomized three-garbage competition cases.')
    parser.add_argument('--workspace', default='/home/hyc/catkin_wa')
    parser.add_argument('--seed', type=int, default=None,
                        help='Random seed. Printed and reusable when omitted.')
    parser.add_argument('--cases', type=int, default=1,
                        help='Number of fresh Gazebo cases to run.')
    parser.add_argument('--gui', choices=['true', 'false'], default='true')
    parser.add_argument('--rviz', choices=['true', 'false'], default='true')
    parser.add_argument('--log-dir', default=None)
    parser.add_argument('--startup-wait', type=float, default=42.0)
    parser.add_argument('--nav-startup-wait', type=float, default=18.0)
    parser.add_argument('--roscore-wait', type=float, default=5.0)
    parser.add_argument('--task-timeout', type=float, default=960.0)
    parser.add_argument('--navigation-timeout', type=float, default=150.0)
    parser.add_argument('--timeout', type=float, default=200.0)
    parser.add_argument('--dropoff-confirm-timeout', type=float, default=8.0)
    args = parser.parse_args(argv)
    if args.cases < 1:
        parser.error('--cases must be at least 1')
    return args


def choose_case(rng):
    rooms = sorted(SLOTS)
    selected = sorted(rng.sample(rooms, 3), key=rooms.index)
    enabled = {name: name in selected for name in SLOTS}
    return {
        'rooms': selected,
        'enabled': enabled,
        'targets': [SLOTS[name][1] for name in selected],
        'trash_bin_fixed': True,
    }


def choose_case_from_seed(seed):
    return choose_case(random.Random(seed))


def scene_launch_command(args, case):
    command = [
        'roslaunch', 'garbage_localization',
        'garbage_localization_random_three_garbage_enhanced_sim.launch',
        'gui:={}'.format(args.gui),
    ]
    for room, (arg_name, _class_name) in SLOTS.items():
        command.append('{}:={}'.format(arg_name, str(case['enabled'][room]).lower()))
    return command


def task_flow_command(args):
    return [
        'rosrun', 'garbage_localization', 'run_robocup_task_flow.py',
        '--scenario', SCENARIO, '--no-launch-stack', '--send-goal',
        '--room-patrol', 'on', '--garbage-competition-mode',
        '--garbage-quota', '3', '--target-classes', 'bottle,paper_ball,box',
        '--dropoff-classes', 'trash_bin',
        '--dropoff-confirm-timeout', str(args.dropoff_confirm_timeout),
        '--object-topic', '/garbage_localization/objects',
        '--navigation-timeout', str(args.navigation_timeout),
        '--timeout', str(args.timeout),
    ]


def validate_result(output, case):
    summaries = fixed.extract_task_summaries(output)
    patrol = [item for item in summaries if item.get('mode') in
              ('four_room_waypoint_patrol', 'room_patrol_move_base_action')]
    fixed.require(patrol, 'No room-patrol JSON summary found')
    summary = patrol[-1]
    fixed.require(summary.get('status') == 'ROOM_PATROL_COMPLETE',
                  'Expected ROOM_PATROL_COMPLETE, got {}'.format(summary.get('status')))
    fixed.require(summary.get('garbage_competition_mode') is True,
                  'Expected competition mode enabled')
    fixed.require(summary.get('requested_targets') == 3 and
                  summary.get('visited_targets') == 3 and
                  summary.get('completed_dropoffs') == 3,
                  'Expected three visited/completed targets, got {}'.format(summary))
    fixed.require((summary.get('final_exit') or {}).get('status') == 'EXIT_REACHED',
                  'Expected EXIT_REACHED, got {}'.format(summary.get('final_exit')))
    audit = summary.get('competition_audit') or {}
    fixed.require(audit.get('status') == 'COMPETITION_AUDIT_PASSED',
                  'Expected COMPETITION_AUDIT_PASSED, got {}'.format(audit))
    targets = summary.get('targets') or []
    rooms = [item.get('room') for item in targets]
    classes = [fixed.target_class_name(item) for item in targets]
    fixed.require(set(rooms) == set(case['rooms']),
                  'Observed rooms {} do not match selected {}'.format(rooms, case['rooms']))
    fixed.require(set(classes).issubset(PICKUP_CLASSES),
                  'Unexpected pickup classes {}'.format(classes))
    fixed.require(len(targets) == 3, 'Expected three target records')
    return {'summary': summary, 'rooms': rooms, 'classes': classes}


def run_case(args, case, log_dir):
    os.makedirs(log_dir, exist_ok=True)
    managed = []
    try:
        fixed.cleanup_competition_regression_processes()
        roscore = fixed.start_process('roscore', args.workspace, ['roscore'], log_dir)
        managed.append(roscore)
        fixed.wait_for_master(args.workspace, args.roscore_wait)
        scene = fixed.start_process('scene', args.workspace,
                                    scene_launch_command(args, case), log_dir)
        managed.append(scene)
        time.sleep(args.startup_wait)
        navigation = fixed.start_process('navigation', args.workspace,
                                         fixed.nav_launch_command(args), log_dir)
        managed.append(navigation)
        time.sleep(args.nav_startup_wait)
        log_path = os.path.join(log_dir, 'task_flow.log')
        command = fixed.shell_command(args.workspace, task_flow_command(args))
        with open(log_path, 'w') as handle:
            process = fixed.subprocess.Popen(['bash', '-lc', command], stdout=handle,
                                             stderr=fixed.subprocess.STDOUT,
                                             start_new_session=True)
            try:
                process.wait(timeout=args.task_timeout)
            except fixed.subprocess.TimeoutExpired:
                fixed.terminate_process_group(process)
                raise RuntimeError('Task flow timed out; log={}'.format(log_path))
        if process.returncode != 0:
            raise RuntimeError('Task flow exit {}; log={}'.format(process.returncode, log_path))
        with open(log_path) as handle:
            result = validate_result(handle.read(), case)
        result['task_log_path'] = log_path
        return result
    finally:
        for item in reversed(managed):
            fixed.terminate_process_group(item.process)
        fixed.cleanup_competition_regression_processes()


def main(argv=None):
    args = parse_args(argv)
    seed = args.seed if args.seed is not None else random.SystemRandom().randrange(1 << 63)
    rng = random.Random(seed)
    results = []
    base = args.log_dir or '/tmp/robocup_random_three_garbage_{}'.format(seed)
    for index in range(args.cases):
        case_seed = rng.randrange(1 << 63)
        case = choose_case_from_seed(case_seed)
        case['seed'] = case_seed
        log_dir = os.path.join(base, 'case_{:03d}_{}'.format(index + 1, '_'.join(case['rooms'])))
        print('RANDOM_CASE seed={} rooms={} log_dir={}'.format(
            case['seed'], ','.join(case['rooms']), log_dir), flush=True)
        result = run_case(args, case, log_dir)
        result['seed'] = case['seed']
        result['selected_rooms'] = case['rooms']
        results.append(result)
    print(json.dumps({'status': 'RANDOM_THREE_GARBAGE_COMPETITION_REGRESSION_PASSED',
                      'seed': seed, 'cases': results}, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
