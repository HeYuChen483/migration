#!/usr/bin/env python3

import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time


DEFAULT_CASES = [
    {
        'name': 'four_occlusion_default_yaw0',
        'scenario': 'four_occlusion',
        'robot_x': None,
        'robot_y': None,
        'robot_yaw': 0.0,
    },
    {
        'name': 'four_occlusion_default_yaw90',
        'scenario': 'four_occlusion',
        'robot_x': None,
        'robot_y': None,
        'robot_yaw': math.pi / 2.0,
    },
    {
        'name': 'four_occlusion_left_yaw0',
        'scenario': 'four_occlusion',
        'robot_x': 6.20,
        'robot_y': 0.45,
        'robot_yaw': 0.0,
    },
    {
        'name': 'four_occlusion_right_yaw0',
        'scenario': 'four_occlusion',
        'robot_x': 7.25,
        'robot_y': 0.45,
        'robot_yaw': 0.0,
    },
]

EXTENDED_CASES = [
    {
        'name': 'four_occlusion_left_yaw90',
        'scenario': 'four_occlusion',
        'robot_x': 6.20,
        'robot_y': 0.45,
        'robot_yaw': math.pi / 2.0,
    },
    {
        'name': 'four_occlusion_inside_yaw90',
        'scenario': 'four_occlusion',
        'robot_x': 6.75,
        'robot_y': 0.75,
        'robot_yaw': math.pi / 2.0,
    },
    {
        'name': 'four_occlusion_right_yaw90',
        'scenario': 'four_occlusion',
        'robot_x': 7.25,
        'robot_y': 0.45,
        'robot_yaw': math.pi / 2.0,
    },
    {
        'name': 'four_occlusion_inside_yaw0',
        'scenario': 'four_occlusion',
        'robot_x': 6.75,
        'robot_y': 0.75,
        'robot_yaw': 0.0,
    },
]

ALL_CASES = DEFAULT_CASES + EXTENDED_CASES

LEGACY_CASE_ALIASES = {
    'four_occlusion_yaw0': 'four_occlusion_default_yaw0',
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run Gazebo-only sequential navigation regressions.')
    parser.add_argument('--workspace', default='/home/hyc/catkin_wa',
                        help='Catkin workspace path.')
    case_names = [case['name'] for case in ALL_CASES]
    case_choices = sorted(
        case_names + list(LEGACY_CASE_ALIASES) + [
            'all', 'default', 'extended', 'all-full'])
    parser.add_argument('--case', action='append', choices=case_choices,
                        help=('Case name to run. Use all/default for the validated '
                              'four-case matrix, extended for extra Gazebo-only '
                              'coverage, all-full for both, or omit for all/default.'))
    parser.add_argument('--visit-count', type=int, default=4)
    parser.add_argument('--startup-wait', type=float, default=38.0)
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--initial-scan-timeout', type=float, default=24.0)
    parser.add_argument('--recovery-scan-timeout', type=float, default=24.0)
    parser.add_argument('--navigation-timeout', type=float, default=150.0)
    parser.add_argument('--command-timeout', type=float, default=720.0)
    parser.add_argument('--minimum-obstacle-distance', type=float, default=0.55)
    parser.add_argument('--minimum-path-obstacle-distance', type=float, default=0.30)
    parser.add_argument('--min-confidence', type=float, default=0.001)
    parser.add_argument('--log-dir', default='/tmp')
    return parser.parse_args(argv)


def selected_cases(names):
    if not names or 'all' in names or 'default' in names:
        return DEFAULT_CASES
    if 'all-full' in names:
        return ALL_CASES
    selected = []
    seen = set()
    expanded = []
    for name in names:
        if name == 'extended':
            expanded.extend(case['name'] for case in EXTENDED_CASES)
        else:
            expanded.append(LEGACY_CASE_ALIASES.get(name, name))
    wanted = set(expanded)
    for case in ALL_CASES:
        if case['name'] in wanted and case['name'] not in seen:
            selected.append(case)
            seen.add(case['name'])
    return selected


def task_flow_command(args, case):
    command = [
        'rosrun', 'garbage_localization', 'run_robocup_task_flow.py',
        '--scenario', case['scenario'],
        '--launch-navigation',
        '--send-goal',
        '--visit-count', str(args.visit_count),
        '--min-targets', '1',
        '--startup-wait', str(args.startup_wait),
        '--timeout', str(args.timeout),
        '--initial-scan-timeout', str(args.initial_scan_timeout),
        '--recovery-scan-timeout', str(args.recovery_scan_timeout),
        '--navigation-timeout', str(args.navigation_timeout),
        '--navigation-min-obstacle-distance', str(args.minimum_obstacle_distance),
        '--navigation-path-min-obstacle-distance', str(args.minimum_path_obstacle_distance),
        '--min-confidence', str(args.min_confidence),
    ]
    for key in ('robot_x', 'robot_y', 'robot_yaw'):
        value = case.get(key)
        if value is not None:
            command.append('{}:={}'.format(key, value))
    return command


def extract_task_summaries(output):
    decoder = json.JSONDecoder()
    summaries = []
    for index, char in enumerate(output):
        if char != '{':
            continue
        try:
            value, _end = decoder.raw_decode(output[index:])
        except ValueError:
            continue
        if isinstance(value, dict) and 'status' in value and 'mode' in value:
            summaries.append(value)
    return summaries


def assert_case_passed(args, case, output):
    summaries = extract_task_summaries(output)
    sequential = [item for item in summaries
                  if item.get('mode') == 'sequential_move_base_action']
    if not sequential:
        raise RuntimeError('No sequential task-flow JSON summary found')
    summary = sequential[0]
    if summary.get('status') != 'SEQUENTIAL_NAVIGATION_COMPLETE':
        raise RuntimeError('Expected SEQUENTIAL_NAVIGATION_COMPLETE, got {}'.format(
            summary.get('status')))
    if summary.get('visited_targets') != args.visit_count:
        raise RuntimeError('Expected visited_targets={}, got {}'.format(
            args.visit_count, summary.get('visited_targets')))
    if summary.get('requested_targets') != args.visit_count:
        raise RuntimeError('Expected requested_targets={}, got {}'.format(
            args.visit_count, summary.get('requested_targets')))
    unsafe = []
    distances = []
    path_distances = []
    for index, target in enumerate(summary.get('targets', []), 1):
        approach = target.get('approach', {})
        distance = approach.get('nearest_map_obstacle_distance_m')
        path_distance = approach.get('path_nearest_map_obstacle_distance_m')
        distances.append(distance)
        path_distances.append(path_distance)
        if distance is None or distance + 1e-9 < args.minimum_obstacle_distance:
            unsafe.append((index, 'endpoint', distance))
        if (path_distance is not None and
                path_distance + 1e-9 < args.minimum_path_obstacle_distance):
            unsafe.append((index, 'path', path_distance))
    if unsafe:
        raise RuntimeError('Safety gate failed: {}'.format(unsafe))
    return {
        'case': case['name'],
        'status': summary.get('status'),
        'visited_targets': summary.get('visited_targets'),
        'navigation_attempts': summary.get('navigation_attempts'),
        'nearest_map_obstacle_distance_m': distances,
        'path_nearest_map_obstacle_distance_m': path_distances,
        'recovery_scan_count': len(summary.get('recovery_scans', [])),
    }


def _terminate_process_group(process, grace_seconds=15.0):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except OSError:
        return
    deadline = time.monotonic() + grace_seconds
    while process.poll() is None and time.monotonic() < deadline:
        time.sleep(0.1)
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except OSError:
            pass
        process.wait()


def _matching_regression_process_pids():
    patterns = (
        'garbage_localization_four_sim.launch',
        'nav_pkg nav.launch',
        'gzserver -e ode',
        'gzclient',
        'yolo_world_detector_node',
        'run_robocup_task_flow.py',
        '/opt/ros/noetic/bin/roscore',
        '/opt/ros/noetic/bin/rosmaster',
    )
    current_pid = os.getpid()
    try:
        current_process_group = os.getpgrp()
    except OSError:
        current_process_group = None
    pids = []
    for name in os.listdir('/proc'):
        try:
            pid = int(name)
        except ValueError:
            continue
        if pid == current_pid:
            continue
        try:
            process_group = os.getpgid(pid)
        except OSError:
            continue
        if process_group == current_process_group:
            continue
        try:
            with open(os.path.join('/proc', name, 'cmdline'), 'rb') as handle:
                raw_command = handle.read()
        except OSError:
            continue
        if not raw_command:
            continue
        command = raw_command.replace(b'\0', b' ').decode(
            'utf-8', errors='replace')
        if any(pattern in command for pattern in patterns):
            pids.append(pid)
    return pids


def cleanup_regression_processes(grace_seconds=12.0):
    pids = _matching_regression_process_pids()
    if not pids:
        return
    process_groups = set()
    for pid in pids:
        try:
            process_groups.add(os.getpgid(pid))
        except OSError:
            pass
    for process_group in process_groups:
        try:
            os.killpg(process_group, signal.SIGTERM)
        except OSError:
            pass
    deadline = time.monotonic() + grace_seconds
    while time.monotonic() < deadline:
        if not _matching_regression_process_pids():
            break
        time.sleep(0.2)
    for pid in _matching_regression_process_pids():
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(1.0)


def run_case(args, case):
    os.makedirs(args.log_dir, exist_ok=True)
    cleanup_regression_processes()
    log_path = os.path.join(args.log_dir,
                            'robocup_seq_nav_{}.log'.format(case['name']))
    command = task_flow_command(args, case)
    shell_command = 'source {workspace}/devel/setup.bash && exec {command}'.format(
        workspace=args.workspace,
        command=' '.join(command))
    timed_out = False
    process = None
    try:
        with open(log_path, 'w') as handle:
            process = subprocess.Popen(
                ['bash', '-lc', shell_command],
                stdout=handle,
                stderr=subprocess.STDOUT,
                universal_newlines=True,
                start_new_session=True)
            try:
                process.wait(timeout=args.command_timeout)
            except subprocess.TimeoutExpired:
                timed_out = True
                handle.write('\nREGRESSION_TIMEOUT after {:.1f}s; terminating process group.\n'.format(
                    args.command_timeout))
                handle.flush()
                _terminate_process_group(process)
        with open(log_path) as handle:
            output = handle.read()
        try:
            result = assert_case_passed(args, case, output)
        except RuntimeError:
            if timed_out:
                raise RuntimeError('Command timed out after {:.1f}s. Log: {}'.format(
                    args.command_timeout, log_path))
            if process is not None and process.returncode != 0:
                raise RuntimeError('Command failed with exit code {}. Log: {}'.format(
                    process.returncode, log_path))
            raise
        result['log_path'] = log_path
        return result
    finally:
        if process is not None:
            poll = getattr(process, 'poll', None)
            if poll is not None and poll() is None:
                _terminate_process_group(process)
        cleanup_regression_processes()


def main():
    args = parse_args()
    results = []
    for case in selected_cases(args.case):
        print('RUNNING {}'.format(case['name']), flush=True)
        result = run_case(args, case)
        results.append(result)
        print('PASSED {}'.format(case['name']), flush=True)
    print(json.dumps({
        'status': 'SEQUENTIAL_NAVIGATION_REGRESSION_PASSED',
        'case_count': len(results),
        'minimum_obstacle_distance_m': args.minimum_obstacle_distance,
        'minimum_path_obstacle_distance_m': args.minimum_path_obstacle_distance,
        'results': results,
    }, indent=2, sort_keys=True))


if __name__ == '__main__':
    main()
