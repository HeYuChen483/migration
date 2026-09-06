#!/usr/bin/env python3
"""Fresh Gazebo/RViz regression for the opt-in three-garbage competition mode."""

from __future__ import print_function

import argparse
import json
import os
import signal
import subprocess
import sys
import time


SCENARIO = 'four_paper_ball'
EXPECTED_GARBAGE_QUOTA = 3
EXPECTED_COMPLETED_ROOMS = ['living_room', 'kitchen', 'bedroom']
EXPECTED_SKIPPED_ROOM = 'dining_room'
EXPECTED_PICKUP_CLASSES = set(['bottle', 'paper_ball', 'box'])
DEFAULT_LOG_DIR_PREFIX = '/tmp/robocup_three_garbage_competition_'


class ManagedProcess(object):
    def __init__(self, name, process, log_path):
        self.name = name
        self.process = process
        self.log_path = log_path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Run one fresh Gazebo/RViz three-garbage competition regression.')
    parser.add_argument('--workspace', default='/home/hyc/catkin_wa',
                        help='Catkin workspace path.')
    parser.add_argument('--log-dir', default=None,
                        help='Directory for roscore/launch/task logs. Defaults to /tmp with timestamp.')
    parser.add_argument('--gui', choices=['true', 'false'], default='true',
                        help='Whether to launch Gazebo GUI. Default: true.')
    parser.add_argument('--rviz', choices=['true', 'false'], default='true',
                        help='Whether to launch RViz through nav.launch when supported. Default: true.')
    parser.add_argument('--startup-wait', type=float, default=42.0,
                        help='Seconds to wait after Gazebo scene launch before task flow.')
    parser.add_argument('--nav-startup-wait', type=float, default=18.0,
                        help='Seconds to wait after navigation/RViz launch before task flow.')
    parser.add_argument('--roscore-wait', type=float, default=5.0)
    parser.add_argument('--task-timeout', type=float, default=960.0,
                        help='Maximum seconds for run_robocup_task_flow.py.')
    parser.add_argument('--navigation-timeout', type=float, default=150.0)
    parser.add_argument('--timeout', type=float, default=200.0)
    parser.add_argument('--dropoff-confirm-timeout', type=float, default=8.0)
    parser.add_argument('--garbage-quota', type=int, default=EXPECTED_GARBAGE_QUOTA)
    parser.add_argument('--object-topic', default='/garbage_localization/objects')
    parser.add_argument('--target-classes', default='bottle,paper_ball,box')
    parser.add_argument('--dropoff-classes', default='trash_bin')
    parser.add_argument('--keep-running-on-failure', action='store_true',
                        help='Leave Gazebo/RViz running if the regression fails.')
    parser.add_argument('--keep-running-on-success', action='store_true',
                        help='Leave Gazebo/RViz running after a successful regression.')
    return parser.parse_args(argv)


def timestamped_log_dir():
    return DEFAULT_LOG_DIR_PREFIX + time.strftime('%Y%m%d_%H%M%S')


def shell_command(workspace, command):
    return 'source {}/devel/setup.bash && exec {}'.format(
        workspace, ' '.join(command))


def start_process(name, workspace, command, log_dir):
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, '{}.log'.format(name))
    with open(log_path, 'w') as handle:
        process = subprocess.Popen(
            ['bash', '-lc', shell_command(workspace, command)],
            stdout=handle,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            start_new_session=True)
    return ManagedProcess(name, process, log_path)


def terminate_process_group(process, grace_seconds=15.0):
    if process is None or process.poll() is not None:
        return
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


def matching_competition_regression_pids():
    patterns = (
        'garbage_localization_four_paper_ball_enhanced_sim.launch',
        'nav_pkg nav.launch',
        'run_robocup_task_flow.py --scenario four_paper_ball',
        'calibrate_four_paper_ball_scene.py',
        'spawn_second_ground_bottle',
        'spawn_bedroom_paper_ball',
        'spawn_living_room_small_box',
        'spawn_living_room_trash_bin',
        'yolo_world_detector_node',
        'garbage_localization_node',
        '/opt/ros/noetic/bin/roscore',
        '/opt/ros/noetic/bin/rosmaster',
        'gzserver',
        'gzclient',
        'rviz',
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
        command = raw_command.replace(b'\0', b' ').decode('utf-8', errors='replace')
        if any(pattern in command for pattern in patterns):
            pids.append(pid)
    return pids


def cleanup_competition_regression_processes(grace_seconds=12.0):
    pids = matching_competition_regression_pids()
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
        if not matching_competition_regression_pids():
            break
        time.sleep(0.2)
    for pid in matching_competition_regression_pids():
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            pass
    time.sleep(1.0)


def wait_for_master(workspace, timeout):
    deadline = time.monotonic() + timeout
    command = ['bash', '-lc', 'source {}/devel/setup.bash && rostopic list >/dev/null 2>&1'.format(workspace)]
    while time.monotonic() < deadline:
        if subprocess.call(command) == 0:
            return
        time.sleep(0.2)
    raise RuntimeError('ROS master did not become available within {:.1f}s'.format(timeout))


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


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def target_dropoff_status(target):
    dropoff = target.get('dropoff', {})
    return target.get('dropoff_status') or dropoff.get('status') or target.get('status')


def target_details(target):
    details = target.get('target') or {}
    if not isinstance(details, dict):
        details = {}
    return details


def target_class_name(target):
    details = target_details(target)
    return (target.get('class_name') or target.get('class') or
            details.get('class_name') or details.get('class'))


def target_confidence(target):
    details = target_details(target)
    return target.get('confidence', details.get('confidence'))


def pre_pickup_dropoff_status(target):
    value = target.get('pre_pickup_dropoff') or target.get('dropoff_confirmation') or {}
    if not isinstance(value, dict):
        return None, None
    return value.get('status'), value.get('source')


def assert_three_garbage_competition_passed(args, output):
    summaries = extract_task_summaries(output)
    room_patrol = [item for item in summaries
                   if item.get('mode') in ('four_room_waypoint_patrol',
                                           'room_patrol_move_base_action')]
    if not room_patrol:
        modes = [item.get('mode') for item in summaries]
        raise RuntimeError(
            'No four-room patrol task-flow JSON summary found; modes={}'.format(modes))
    summary = room_patrol[-1]
    final_exit = summary.get('final_exit') or {}
    require(summary.get('status') == 'ROOM_PATROL_COMPLETE',
            'Expected ROOM_PATROL_COMPLETE, got {} final_exit={} stop_reason={}'.format(
                summary.get('status'), final_exit, summary.get('stop_reason')))
    require(summary.get('garbage_competition_mode') is True,
            'Expected garbage_competition_mode=true, got {}'.format(
                summary.get('garbage_competition_mode')))
    require(summary.get('requested_targets') == args.garbage_quota,
            'Expected requested_targets={}, got {}'.format(
                args.garbage_quota, summary.get('requested_targets')))
    require(summary.get('visited_targets') == args.garbage_quota,
            'Expected visited_targets={}, got {}'.format(
                args.garbage_quota, summary.get('visited_targets')))
    require(summary.get('completed_dropoffs') == args.garbage_quota,
            'Expected completed_dropoffs={}, got {}'.format(
                args.garbage_quota, summary.get('completed_dropoffs')))
    require(final_exit.get('status') == 'EXIT_REACHED',
            'Expected final_exit.status EXIT_REACHED, got {}'.format(
                final_exit.get('status')))

    audit = summary.get('competition_audit') or {}
    require(audit.get('status') == 'COMPETITION_AUDIT_PASSED',
            'Expected COMPETITION_AUDIT_PASSED, got {}'.format(audit.get('status')))

    targets = summary.get('targets', [])
    require(len(targets) == args.garbage_quota,
            'Expected {} target summaries, got {}'.format(args.garbage_quota, len(targets)))
    completed_rooms = summary.get('completed_rooms', [])
    require(completed_rooms == EXPECTED_COMPLETED_ROOMS,
            'Expected completed rooms {}, got {}'.format(
                EXPECTED_COMPLETED_ROOMS, completed_rooms))

    skipped = summary.get('skipped_rooms_after_quota', [])
    skipped_rooms = [item.get('room') for item in skipped]
    require(skipped_rooms == [EXPECTED_SKIPPED_ROOM],
            'Expected skipped room {}, got {}'.format(EXPECTED_SKIPPED_ROOM, skipped_rooms))
    require(skipped[0].get('status') == 'SKIPPED_GARBAGE_QUOTA_REACHED',
            'Expected skipped status SKIPPED_GARBAGE_QUOTA_REACHED, got {}'.format(
                skipped[0].get('status')))

    classes = []
    dropoff_failures = []
    preconfirm_failures = []
    for index, target in enumerate(targets, 1):
        class_name = target_class_name(target)
        classes.append(class_name)
        status = target_dropoff_status(target)
        if status != 'DROPOFF_REACHED':
            dropoff_failures.append((index, target.get('room'), class_name, status))
        pre_status, pre_source = pre_pickup_dropoff_status(target)
        if pre_status != 'DROPOFF_CONFIRMED' or pre_source != 'current_perception':
            preconfirm_failures.append((index, target.get('room'), class_name, pre_status, pre_source))
    require(set(classes).issubset(EXPECTED_PICKUP_CLASSES),
            'Unexpected pickup target classes: {}'.format(classes))
    require(EXPECTED_SKIPPED_ROOM not in [target.get('room') for target in targets],
            'Skipped room {} should not have a target summary'.format(EXPECTED_SKIPPED_ROOM))
    require(not dropoff_failures,
            'Targets without DROPOFF_REACHED: {}'.format(dropoff_failures))
    require(not preconfirm_failures,
            'Targets without current-perception pre-pickup dropoff confirmation: {}'.format(
                preconfirm_failures))

    return {
        'status': summary.get('status'),
        'garbage_competition_mode': summary.get('garbage_competition_mode'),
        'requested_targets': summary.get('requested_targets'),
        'visited_targets': summary.get('visited_targets'),
        'completed_dropoffs': summary.get('completed_dropoffs'),
        'completed_rooms': completed_rooms,
        'skipped_rooms_after_quota': skipped,
        'final_exit': final_exit.get('status'),
        'competition_audit': audit.get('status'),
        'targets': [
            {
                'index': index,
                'room': target.get('room'),
                'class_name': target_class_name(target),
                'confidence': target_confidence(target),
                'dropoff_status': target_dropoff_status(target),
                'pre_pickup_dropoff': pre_pickup_dropoff_status(target),
            }
            for index, target in enumerate(targets, 1)
        ],
    }


def scene_launch_command(args):
    return [
        'roslaunch', 'garbage_localization',
        'garbage_localization_four_paper_ball_enhanced_sim.launch',
        'gui:={}'.format(args.gui),
    ]


def nav_launch_command(args):
    command = ['roslaunch', 'nav_pkg', 'nav.launch']
    if args.rviz == 'false':
        command.append('rviz:=false')
    return command


def task_flow_command(args):
    return [
        'rosrun', 'garbage_localization', 'run_robocup_task_flow.py',
        '--scenario', SCENARIO,
        '--no-launch-stack',
        '--send-goal',
        '--room-patrol', 'on',
        '--garbage-competition-mode',
        '--garbage-quota', str(args.garbage_quota),
        '--target-classes', args.target_classes,
        '--dropoff-classes', args.dropoff_classes,
        '--dropoff-confirm-timeout', str(args.dropoff_confirm_timeout),
        '--object-topic', args.object_topic,
        '--navigation-timeout', str(args.navigation_timeout),
        '--timeout', str(args.timeout),
    ]


def run_task_command(args, log_dir):
    log_path = os.path.join(log_dir, 'task_flow.log')
    command = shell_command(args.workspace, task_flow_command(args))
    timed_out = False
    with open(log_path, 'w') as handle:
        process = subprocess.Popen(
            ['bash', '-lc', command],
            stdout=handle,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            start_new_session=True)
        try:
            process.wait(timeout=args.task_timeout)
        except subprocess.TimeoutExpired:
            timed_out = True
            handle.write('\nTASK_FLOW_TIMEOUT after {:.1f}s; terminating process group.\n'.format(
                args.task_timeout))
            handle.flush()
            terminate_process_group(process)
    with open(log_path) as handle:
        output = handle.read()
    if timed_out:
        raise RuntimeError('Task flow timed out after {:.1f}s. Log: {}'.format(
            args.task_timeout, log_path))
    if process.returncode != 0:
        raise RuntimeError('Task flow failed with exit code {}. Log: {}'.format(
            process.returncode, log_path))
    result = assert_three_garbage_competition_passed(args, output)
    result['task_log_path'] = log_path
    return result


def run_regression(args):
    log_dir = args.log_dir or timestamped_log_dir()
    os.makedirs(log_dir, exist_ok=True)
    managed = []
    success = False
    try:
        cleanup_competition_regression_processes()
        roscore = start_process('roscore', args.workspace, ['roscore'], log_dir)
        managed.append(roscore)
        wait_for_master(args.workspace, args.roscore_wait)
        scene = start_process('scene', args.workspace, scene_launch_command(args), log_dir)
        managed.append(scene)
        print('SCENE_LAUNCHED log={}'.format(scene.log_path), flush=True)
        time.sleep(args.startup_wait)
        navigation = start_process('navigation', args.workspace, nav_launch_command(args), log_dir)
        managed.append(navigation)
        print('NAVIGATION_LAUNCHED log={}'.format(navigation.log_path), flush=True)
        time.sleep(args.nav_startup_wait)
        result = run_task_command(args, log_dir)
        result['log_dir'] = log_dir
        result['scene_log_path'] = scene.log_path
        result['navigation_log_path'] = navigation.log_path
        result['roscore_log_path'] = roscore.log_path
        success = True
        return result
    finally:
        keep_running = ((success and args.keep_running_on_success) or
                        ((not success) and args.keep_running_on_failure))
        if not keep_running:
            for item in reversed(managed):
                terminate_process_group(item.process)
            cleanup_competition_regression_processes()
        else:
            print('LEAVING_STACK_RUNNING log_dir={}'.format(log_dir), flush=True)


def main(argv=None):
    args = parse_args(argv)
    result = run_regression(args)
    print(json.dumps({
        'status': 'THREE_GARBAGE_COMPETITION_REGRESSION_PASSED',
        'scenario': SCENARIO,
        'result': result,
    }, indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
