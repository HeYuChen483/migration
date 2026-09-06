#!/usr/bin/env python3
"""Run the Gazebo-only RoboCup bottle localization baseline.

This helper intentionally keeps the current usable baseline simple: it launches the
existing Gazebo + YOLO-World + garbage_localization stack, optionally runs the
strict Gazebo acceptance evaluator, and never touches the real robot/camera/arm.
"""

from __future__ import print_function

import argparse
import os
import signal
import subprocess
import sys
import time


SCENARIOS = {
    'single': {
        'launch': 'garbage_localization_sim.launch',
        'models': ['ground_red_bottle'],
    },
    'multi': {
        'launch': 'garbage_localization_multi_sim.launch',
        'models': ['ground_red_bottle', 'ground_red_bottle_second'],
    },
    'partial_occlusion': {
        'launch': 'garbage_localization_multi_sim.launch',
        'launch_args': ['scenario:=partial_occlusion'],
        'models': ['ground_red_bottle', 'ground_red_bottle_second'],
    },
    'three': {
        'launch': 'garbage_localization_three_sim.launch',
        'models': [
            'ground_red_bottle',
            'ground_red_bottle_second',
            'ground_red_bottle_third',
        ],
    },
    'four': {
        'launch': 'garbage_localization_four_sim.launch',
        'models': [
            'ground_red_bottle',
            'ground_red_bottle_second',
            'ground_red_bottle_third',
            'ground_red_bottle_fourth',
        ],
    },
    'four_occlusion': {
        'launch': 'garbage_localization_four_sim.launch',
        'launch_args': [
            'garbage_x:=1.75', 'garbage_y:=-0.16',
            'second_garbage_x:=2.18', 'second_garbage_y:=-0.08',
            'third_garbage_x:=1.92', 'third_garbage_y:=0.18',
            'fourth_garbage_x:=2.35', 'fourth_garbage_y:=0.26',
        ],
        'models': [
            'ground_red_bottle',
            'ground_red_bottle_second',
            'ground_red_bottle_third',
            'ground_red_bottle_fourth',
        ],
    },
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run the Gazebo-only RoboCup bottle localization baseline.')
    parser.add_argument(
        '--scenario', choices=sorted(SCENARIOS), default='three',
        help='Gazebo bottle scene to launch. Default: three')
    parser.add_argument(
        '--gui', action='store_true',
        help='Show Gazebo GUI. Default: headless')
    parser.add_argument(
        '--check', action='store_true',
        help='Run strict Gazebo acceptance after the stack starts.')
    parser.add_argument(
        '--samples', type=int, default=20,
        help='Acceptance sample count when --check is used. Default: 20')
    parser.add_argument(
        '--timeout', type=float, default=60.0,
        help='Acceptance timeout in seconds when --check is used. Default: 60')
    parser.add_argument(
        '--startup-wait', type=float, default=12.0,
        help='Seconds to wait before starting acceptance. Default: 12')
    parser.add_argument(
        'launch_overrides', nargs='*',
        help='Extra roslaunch args, e.g. garbage_x:=2.0 garbage_y:=0.2')
    return parser.parse_args()


def sourced_environment(workspace):
    setup = os.path.join(workspace, 'devel', 'setup.bash')
    command = 'source "{}" >/dev/null 2>&1 && env -0'.format(setup)
    output = subprocess.check_output(['bash', '-lc', command])
    env = {}
    for item in output.split(b'\0'):
        if not item:
            continue
        key, _, value = item.partition(b'=')
        env[key.decode()] = value.decode()
    return env


def run_command(command, env):
    print('+ {}'.format(' '.join(command)))
    return subprocess.Popen(command, env=env, preexec_fn=os.setsid)


def stop_process(process):
    if process.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGINT)
        process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        process.wait(timeout=5.0)


def main():
    args = parse_args()
    workspace = '/home/hyc/catkin_wa'
    scenario = SCENARIOS[args.scenario]
    env = sourced_environment(workspace)

    launch_args = [
        'roslaunch', 'garbage_localization', scenario['launch'],
        'gui:={}'.format(str(args.gui).lower()),
    ]
    launch_args.extend(scenario.get('launch_args', []))
    launch_args.extend(args.launch_overrides)

    stack = run_command(launch_args, env)
    try:
        if not args.check:
            print('Gazebo-only baseline is running. Press Ctrl-C to stop.')
            while stack.poll() is None:
                time.sleep(1.0)
            return stack.returncode

        time.sleep(max(0.0, args.startup_wait))
        model_names = ','.join(scenario['models'])
        check_args = [
            'rosrun', 'garbage_localization', 'evaluate_multi_bottle.py',
            '_model_names:=[{}]'.format(model_names),
            '_sample_count:={}'.format(args.samples),
            '_timeout_sec:={}'.format(args.timeout),
            '_reference_model:=wpb_home',
            '_reference_frame:=base_link',
            '_maximum_median_error_m:=0.05',
            '_minimum_detection_complete_ratio:=0.90',
            '_minimum_localization_complete_ratio:=0.90',
        ]
        result = subprocess.call(check_args, env=env)
        return result
    finally:
        stop_process(stack)


if __name__ == '__main__':
    sys.exit(main())
