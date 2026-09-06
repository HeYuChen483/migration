#!/usr/bin/env python3
"""Minimal Gazebo-only RoboCup task flow.

The flow is intentionally simple for the current usable baseline:
  1. launch or attach to the Gazebo garbage localization stack,
  2. wait for stable /garbage_localization/objects output,
  3. select one target from the configured same-priority class set,
  4. compute an approach pose in the localization output frame,
  5. either print the goal, publish it, or send it to move_base.

By default this is a safe dry-run and does not move the real robot.
"""

from __future__ import print_function

import argparse
import heapq
import json
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import rospy
import tf2_geometry_msgs
import tf2_ros
from geometry_msgs.msg import Point, PoseStamped
from garbage_localization.msg import GarbageObjectArray


SCENARIOS = {
    'single': {
        'launch': 'garbage_localization_sim.launch',
        'expected_targets': 1,
    },
    'multi': {
        'launch': 'garbage_localization_multi_sim.launch',
        'expected_targets': 2,
    },
    'partial_occlusion': {
        'launch': 'garbage_localization_multi_sim.launch',
        'launch_args': ['scenario:=partial_occlusion'],
        'expected_targets': 2,
    },
    'three': {
        'launch': 'garbage_localization_three_sim.launch',
        'expected_targets': 3,
    },
    'four': {
        'launch': 'garbage_localization_four_sim.launch',
        'expected_targets': 4,
    },
    'four_occlusion': {
        'launch': 'garbage_localization_four_sim.launch',
        'launch_args': [
            'garbage_x:=6.75', 'garbage_y:=1.85', 'garbage_z:=0.02',
            'second_garbage_x:=5.42', 'second_garbage_y:=2.04', 'second_garbage_z:=0.52',
            'third_garbage_x:=5.68', 'third_garbage_y:=2.16', 'third_garbage_z:=0.52',
            'fourth_garbage_x:=6.35', 'fourth_garbage_y:=3.10', 'fourth_garbage_z:=0.52',
        ],
        'expected_targets': 4,
    },
    'four_paper_ball': {
        'launch': 'garbage_localization_four_paper_ball_enhanced_sim.launch',
        'expected_targets': 4,
        'navigation_config': '$(find garbage_localization)/config/enhanced_sim.yaml',
    },
}
FOUR_PAPER_BALL_ROOM_REGIONS = {
    # Conservative task-layer regions for the current competition-style Gazebo
    # scene.  They prevent doorway/near-robot false positives from completing a
    # different room before that room's own waypoint is reached.
    'living_room': {'min_x': 4.40, 'max_x': 8.90, 'min_y': 0.80, 'max_y': 4.20},
    'kitchen': {'min_x': 5.30, 'max_x': 8.90, 'min_y': 4.60, 'max_y': 8.80},
    'bedroom': {'min_x': 0.40, 'max_x': 4.20, 'min_y': 1.20, 'max_y': 4.80},
    'dining_room': {'min_x': 0.40, 'max_x': 4.60, 'min_y': 4.60, 'max_y': 8.80},
}

FOUR_PAPER_BALL_DROPOFF_REGION = {
    'name': 'living_room_trash_bin',
    'center_x': 4.157,
    'center_y': 4.770,
    'radius_m': 1.20,
}

def point_in_rect(x, y, region):
    return (region['min_x'] <= x <= region['max_x'] and
            region['min_y'] <= y <= region['max_y'])


def point_in_radius(x, y, region):
    return (math.hypot(x - region['center_x'], y - region['center_y']) <=
            region['radius_m'])


def pose_position_xy(pose):
    return pose.pose.position.x, pose.pose.position.y


def four_paper_ball_enabled(args):
    return getattr(args, 'scenario', None) == 'four_paper_ball'


COMPETITION_GARBAGE_CLASSES = ['bottle', 'paper_ball', 'box']


def garbage_competition_mode_from_args(args):
    return bool(getattr(args, 'garbage_competition_mode', False))


def garbage_quota_from_args(args):
    return max(1, int(getattr(args, 'garbage_quota', 3)))


def requested_garbage_target_count(args):
    if garbage_competition_mode_from_args(args):
        return garbage_quota_from_args(args)
    return int(getattr(args, 'visit_count', 1))


def completed_dropoff_count(targets):
    return len([item for item in targets
                if item.get('status') == 'DROPOFF_REACHED'])


def completed_target_count(args, targets):
    if dropoff_enabled_from_args(args):
        return completed_dropoff_count(targets)
    return len([item for item in targets
                if item.get('status') in ('READY_FOR_PICKUP', 'DROPOFF_REACHED')])


def unique_patrol_rooms(patrol_waypoints):
    rooms = []
    seen = set()
    for waypoint in patrol_waypoints:
        room = waypoint.get('room')
        if not room or room in seen:
            continue
        rooms.append({'room': room, 'waypoint': waypoint.get('name')})
        seen.add(room)
    return rooms


def skipped_rooms_after_quota(args, patrol_waypoints, completed_rooms, room_visits):
    if not garbage_competition_mode_from_args(args):
        return []
    visited_rooms = set(item.get('room') for item in room_visits)
    completed = set(completed_rooms)
    skipped = []
    for item in unique_patrol_rooms(patrol_waypoints):
        room = item['room']
        if room in completed or room in visited_rooms:
            continue
        skipped.append({
            'room': room,
            'waypoint': item['waypoint'],
            'status': 'SKIPPED_GARBAGE_QUOTA_REACHED',
            'reason': 'garbage_quota_reached',
            'garbage_quota': garbage_quota_from_args(args),
        })
    return skipped


def parse_target_classes(raw_classes, fallback_class):
    raw = raw_classes if raw_classes is not None else fallback_class
    if isinstance(raw, (list, tuple)):
        items = raw
    else:
        items = str(raw).split(',')
    result = []
    seen = set()
    for item in items:
        item = str(item).strip()
        if not item or item in seen:
            continue
        result.append(item)
        seen.add(item)
    return result


def target_classes_from_args(args):
    return parse_target_classes(
        getattr(args, 'target_classes', None),
        getattr(args, 'target_class', 'bottle'))


def target_class_label(args):
    return ','.join(target_classes_from_args(args))


def dropoff_classes_from_args(args):
    return parse_target_classes(
        getattr(args, 'dropoff_classes', 'trash_bin'), 'trash_bin')


def dropoff_class_label(args):
    return ','.join(dropoff_classes_from_args(args))


def dropoff_enabled_from_args(args):
    if getattr(args, 'disable_dropoff', False):
        return False
    if getattr(args, 'enable_dropoff', False):
        return True
    return (getattr(args, 'scenario', None) == 'four_paper_ball' or
            target_classes_from_args(args) != ['bottle'])


def competition_flow_required_from_args(args):
    return (getattr(args, 'scenario', None) == 'four_paper_ball' or
            getattr(args, 'enable_dropoff', False) or
            target_classes_from_args(args) != ['bottle'])


def parse_args():
    parser = argparse.ArgumentParser(
        description='Run the minimal Gazebo-only RoboCup garbage task flow.')
    parser.add_argument(
        '--scenario', choices=sorted(SCENARIOS), default='three',
        help='Gazebo bottle scene to launch or expect. Default: three')
    parser.add_argument(
        '--gui', action='store_true',
        help='Show Gazebo GUI when launching the stack. Default: headless')
    parser.add_argument(
        '--no-launch-stack', action='store_true',
        help='Attach to an already running localization stack instead of launching it.')
    parser.add_argument(
        '--launch-navigation', action='store_true',
        help='Also launch a Gazebo-compatible navigation-only move_base stack.')
    parser.add_argument(
        '--startup-wait', type=float, default=12.0,
        help='Seconds to wait after launching the stack before listening. Default: 12')
    parser.add_argument(
        '--timeout', type=float, default=45.0,
        help='Seconds to wait for a usable localization message. Default: 45')
    parser.add_argument(
        '--object-topic', default='/garbage_localization/objects',
        help='GarbageObjectArray topic. Default: /garbage_localization/objects')
    parser.add_argument(
        '--target-class', default='bottle',
        help='Legacy single target class. Default: bottle')
    parser.add_argument(
        '--target-classes', default=None,
        help=('Comma-separated same-priority target classes. Defaults to '
              '--target-class; e.g. bottle,paper_ball'))
    parser.add_argument(
        '--min-targets', type=int, default=None,
        help='Minimum target count before selecting. Default: scenario target count')
    parser.add_argument(
        '--selector', choices=['nearest', 'confidence'], default='nearest',
        help=('Target selection rule applied across all configured target '
              'classes with no class priority. Default: nearest'))
    parser.add_argument(
        '--min-confidence', type=float, default=0.0,
        help='Minimum detector confidence for target candidates. Default: 0.0')
    parser.add_argument(
        '--visit-count', type=int, default=1,
        help='Number of unique targets to send sequentially with --send-goal. Default: 1')
    parser.add_argument(
        '--garbage-competition-mode', action='store_true',
        help=('Opt-in rule-mode for four-room garbage cleanup: patrol existing rooms, '
              'complete up to --garbage-quota successful garbage drop-offs, then exit.'))
    parser.add_argument(
        '--garbage-quota', type=int, default=3,
        help='Successful garbage drop-offs required in --garbage-competition-mode. Default: 3')
    parser.add_argument(
        '--target-dedup-distance', type=float, default=0.45,
        help='Minimum navigation-frame distance for treating targets as distinct. Default: 0.45')
    parser.add_argument(
        '--approach-distance', type=float, default=0.65,
        help='Distance to stop before the selected garbage target, in meters. Default: 0.65')
    parser.add_argument(
        '--dropoff-classes', default='trash_bin',
        help=('Comma-separated trash-bin/drop-off classes. Kept separate from '
              'garbage target classes; default: trash_bin'))
    parser.add_argument(
        '--dropoff-approach-distance', type=float, default=0.75,
        help='Distance to stop before the localized trash bin, in meters. Default: 0.75')
    parser.add_argument(
        '--enable-dropoff', action='store_true',
        help=('Force trash-bin navigation after pickup. The four_paper_ball '
              'competition scene and multi-class garbage targets enable it automatically.'))
    parser.add_argument(
        '--pickup-action', choices=['dry-run', 'none'], default='dry-run',
        help=('Task-layer pickup action hook. dry-run records the intended '
              'pickup without commanding hardware; none disables the hook. '
              'Default: dry-run'))
    parser.add_argument(
        '--dropoff-action', choices=['dry-run', 'none'], default='dry-run',
        help=('Task-layer drop-off action hook. dry-run records the intended '
              'drop-off without commanding hardware; none disables the hook. '
              'Default: dry-run'))
    parser.add_argument(
        '--dropoff-confirm-timeout', type=float, default=8.0,
        help=('Seconds to confirm/cache the localized trash-bin before pickup '
              'navigation starts. Default: 8'))
    parser.add_argument(
        '--disable-dropoff', action='store_true',
        help='Disable trash-bin navigation after each successful pickup target.')
    parser.add_argument(
        '--final-exit', default='auto',
        help=("Final egress waypoint after all garbage has been dropped off. "
              "Use auto for the non-start opening in four-room Gazebo, none to disable, "
              "or name:x:y:yaw_deg. Default: auto"))
    parser.add_argument(
        '--final-exit-timeout', type=float, default=90.0,
        help='Seconds to wait for each final exit move_base goal. Default: 90')
    parser.add_argument(
        '--final-exit-attempts', type=int, default=4,
        help=('Maximum same-pose final exit move_base attempts after all dropoffs. '
              'Clears costmaps before retry attempts. Default: 4'))
    parser.add_argument(
        '--allow-behind', action='store_true',
        help='Allow targets behind the robot. Default: front targets only')
    parser.add_argument(
        '--publish-goal', action='store_true',
        help='Publish the approach pose once to /move_base_simple/goal.')
    parser.add_argument(
        '--send-goal', action='store_true',
        help='Send the approach pose to the move_base action server and wait.')
    parser.add_argument(
        '--move-base-action', default='move_base',
        help='move_base action name when --send-goal is used. Default: move_base')
    parser.add_argument(
        '--navigation-frame', default='map',
        help='Frame used for move_base goals. Default: map')
    parser.add_argument(
        '--navigation-transform-timeout', type=float, default=5.0,
        help='Seconds to wait for target-frame transform. Default: 5')
    parser.add_argument(
        '--navigation-timeout', type=float, default=60.0,
        help='Seconds to wait for move_base result. Default: 60')
    parser.add_argument(
        '--disable-initial-scan', action='store_true',
        help='Disable in-place scan recovery before the first usable detection.')
    parser.add_argument(
        '--initial-scan-timeout', type=float, default=10.0,
        help='Seconds to wait for detections after each initial scan turn. Default: 10')
    parser.add_argument(
        '--disable-recovery-scan', action='store_true',
        help='Disable in-place navigation scan recovery after sequential re-perception timeout.')
    parser.add_argument(
        '--recovery-scan-timeout', type=float, default=12.0,
        help='Seconds to wait for detections after each recovery scan turn. Default: 12')
    parser.add_argument(
        '--navigation-map',
        default='/home/hyc/catkin_wa/src/wpr_simulation/maps/map.yaml',
        help='Static map yaml used to avoid unreachable navigation goals.')
    parser.add_argument(
        '--search-waypoints', default='auto',
        help=("Task-layer search waypoints for sequential bottle search. "
              "Use auto for four-room Gazebo defaults, none to disable, or "
              "name:x:y:yaw_deg[,name:x:y:yaw_deg...]. Default: auto"))
    parser.add_argument(
        '--search-waypoint-timeout', type=float, default=90.0,
        help='Seconds to wait for each task-layer search waypoint. Default: 90')
    parser.add_argument(
        '--search-reposition-path-length', type=float, default=4.0,
        help=("Routed path length above which sequential search first moves to "
              "the nearest room waypoint and refreshes perception. Use 0 to "
              "disable. Default: 4.0"))
    parser.add_argument(
        '--room-patrol', default='auto',
        help=("Use four-room competition waypoint patrol. Auto enables it for "
              "four_paper_ball multi-target move_base runs; use off/none to "
              "keep legacy sequential search. Default: auto"))
    parser.add_argument(
        '--navigation-min-obstacle-distance', type=float, default=0.55,
        help='Minimum map distance from goal to occupied cells. Default: 0.55')
    parser.add_argument(
        '--navigation-path-min-obstacle-distance', type=float, default=0.30,
        help=('Minimum routed path clearance from occupied map cells. '
              'Default: 0.30'))
    parser.add_argument(
        '--disable-navigation-map-filter', action='store_true',
        help='Do not skip map-unsafe candidates before publishing/sending goals.')
    parser.add_argument(
        '--disable-navigation-path-filter', action='store_true',
        help='Do not skip candidates without a routed map-safe path.')
    parser.add_argument(
        '--disable-indoor-route-filter', action='store_true',
        help=('Do not reject task-layer candidate paths that leave the indoor '
              'competition area through the start entrance.'))
    parser.add_argument(
        'launch_overrides', nargs='*',
        help='Extra roslaunch args, e.g. garbage_x:=2.0 garbage_y:=0.2')
    args = parser.parse_args()
    args.target_classes = parse_target_classes(
        args.target_classes, args.target_class)
    args.dropoff_classes = parse_target_classes(args.dropoff_classes, 'trash_bin')
    if not args.target_classes:
        parser.error('--target-classes/--target-class must include at least one class')
    if args.enable_dropoff and args.disable_dropoff:
        parser.error('--enable-dropoff and --disable-dropoff are mutually exclusive')
    if dropoff_enabled_from_args(args) and not args.dropoff_classes:
        parser.error('--dropoff-classes must include at least one class when drop-off is enabled')
    if args.publish_goal and args.send_goal:
        parser.error('--publish-goal and --send-goal are mutually exclusive')
    if args.min_confidence < 0.0:
        parser.error('--min-confidence must be non-negative')
    if args.visit_count < 1:
        parser.error('--visit-count must be at least 1')
    if args.garbage_quota < 1:
        parser.error('--garbage-quota must be at least 1')
    if args.garbage_competition_mode:
        if args.scenario != 'four_paper_ball':
            parser.error('--garbage-competition-mode currently requires --scenario four_paper_ball')
        if args.disable_dropoff:
            parser.error('--garbage-competition-mode requires drop-off to stay enabled')
        args.target_classes = [item for item in COMPETITION_GARBAGE_CLASSES]
        args.visit_count = args.garbage_quota
    if args.visit_count != 1 and not args.send_goal:
        parser.error('--visit-count greater than 1 requires --send-goal')
    if args.publish_goal and args.visit_count != 1:
        parser.error('--publish-goal only supports --visit-count 1')
    if args.target_dedup_distance < 0.0:
        parser.error('--target-dedup-distance must be non-negative')
    if args.approach_distance < 0.0:
        parser.error('--approach-distance must be non-negative')
    if args.dropoff_approach_distance < 0.0:
        parser.error('--dropoff-approach-distance must be non-negative')
    if args.dropoff_confirm_timeout < 0.0:
        parser.error('--dropoff-confirm-timeout must be non-negative')
    if args.final_exit_timeout < 0.0:
        parser.error('--final-exit-timeout must be non-negative')
    if args.final_exit_attempts < 1:
        parser.error('--final-exit-attempts must be at least 1')
    if args.search_waypoint_timeout < 0.0:
        parser.error('--search-waypoint-timeout must be non-negative')
    if args.search_reposition_path_length < 0.0:
        parser.error('--search-reposition-path-length must be non-negative')
    return args


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


def start_process(command, env):
    print('+ {}'.format(' '.join(command)))
    return subprocess.Popen(command, env=env, preexec_fn=os.setsid)


def wait_for_ros_master(env, timeout=10.0):
    deadline = time.monotonic() + timeout
    command = ['rosnode', 'list']
    while time.monotonic() < deadline:
        try:
            subprocess.check_output(command, env=env, stderr=subprocess.STDOUT)
            return
        except subprocess.CalledProcessError:
            time.sleep(0.2)
    raise RuntimeError('ROS master did not become available within {:.1f}s'.format(
        timeout))


def start_roscore(env):
    process = start_process(['roscore'], env)
    wait_for_ros_master(env)
    return process


def set_ros_param(env, name, value):
    subprocess.check_call(['rosparam', 'set', name, str(value).lower()], env=env)


def _process_parent_map():
    parents = {}
    for name in os.listdir('/proc'):
        try:
            pid = int(name)
        except ValueError:
            continue
        try:
            with open(os.path.join('/proc', name, 'stat')) as handle:
                fields = handle.read().split()
            parents[pid] = int(fields[3])
        except (OSError, IndexError, ValueError):
            continue
    return parents


def _descendant_pids(root_pid):
    parents = _process_parent_map()
    descendants = []
    stack = [root_pid]
    while stack:
        parent = stack.pop()
        children = [pid for pid, ppid in parents.items() if ppid == parent]
        descendants.extend(children)
        stack.extend(children)
    return descendants


def _process_groups_for_tree(root_pid):
    process_groups = []
    for pid in [root_pid] + _descendant_pids(root_pid):
        try:
            process_group = os.getpgid(pid)
        except OSError:
            continue
        if process_group not in process_groups:
            process_groups.append(process_group)
    return process_groups


def _signal_process_tree(root_pid, sig):
    for process_group in _process_groups_for_tree(root_pid):
        try:
            os.killpg(process_group, sig)
        except OSError:
            pass


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    _signal_process_tree(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10.0)
        return
    except subprocess.TimeoutExpired:
        pass
    _signal_process_tree(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass
    _signal_process_tree(process.pid, signal.SIGKILL)
    try:
        process.wait(timeout=2.0)
    except subprocess.TimeoutExpired:
        pass


def launch_localization_stack(args, env):
    scenario = SCENARIOS[args.scenario]
    command = [
        'roslaunch', 'garbage_localization', scenario['launch'],
        'gui:={}'.format(str(args.gui).lower()),
    ]
    if args.launch_navigation:
        navigation_config = scenario.get(
            'navigation_config',
            '$(find garbage_localization)/config/sim_navigation.yaml')
        command.append('localization_config:={}'.format(navigation_config))
    command.extend(scenario.get('launch_args', []))
    command.extend(args.launch_overrides)
    return start_process(command, env)


def launch_override_value(args, name, default=None):
    prefix = '{}:='.format(name)
    for item in args.launch_overrides:
        if item.startswith(prefix):
            return item[len(prefix):]
    return default


def navigation_launch_args(args):
    """Keep AMCL's initial pose synchronized with the Gazebo robot spawn."""
    mapping = [
        ('robot_x', 'initial_pose_x'),
        ('robot_y', 'initial_pose_y'),
        ('robot_yaw', 'initial_pose_a'),
    ]
    values = []
    for source_name, navigation_name in mapping:
        value = launch_override_value(args, source_name)
        if value is not None:
            values.append('{}:={}'.format(navigation_name, value))
    return values


def launch_navigation_stack(args, env):
    command = ['roslaunch', 'nav_pkg', 'nav.launch']
    command.extend(navigation_launch_args(args))
    return start_process(command, env)


def finite(value):
    return math.isfinite(float(value))


def yaw_to_quaternion(yaw):
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw * 0.5),
        'w': math.cos(yaw * 0.5),
    }


def pose_to_dict(pose):
    return {
        'frame_id': pose.header.frame_id,
        'position': {
            'x': pose.pose.position.x,
            'y': pose.pose.position.y,
            'z': pose.pose.position.z,
        },
        'orientation': {
            'x': pose.pose.orientation.x,
            'y': pose.pose.orientation.y,
            'z': pose.pose.orientation.z,
            'w': pose.pose.orientation.w,
        },
    }


class StaticMapSafety(object):
    OCCUPIED_THRESHOLD = 89
    PATH_START_SKIP_M = 0.30

    def __init__(self, yaml_path, minimum_obstacle_distance,
                 path_minimum_obstacle_distance=None,
                 forbidden_path_regions=None):
        self.yaml_path = yaml_path
        self.minimum_obstacle_distance = minimum_obstacle_distance
        self.path_minimum_obstacle_distance = (
            path_minimum_obstacle_distance
            if path_minimum_obstacle_distance is not None
            else minimum_obstacle_distance)
        self.image_path = None
        self.resolution = None
        self.origin = None
        self.width = 0
        self.height = 0
        self.pixels = []
        self.forbidden_path_regions = list(forbidden_path_regions or [])
        self._occupied_cache = {}
        self._blocked_cache = None
        self._load()

    def _load(self):
        values = {}
        with open(self.yaml_path) as handle:
            for line in handle:
                line = line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue
                key, value = line.split(':', 1)
                values[key.strip()] = value.strip()
        image = values['image'].strip('"\'')
        if not os.path.isabs(image):
            image = os.path.join(os.path.dirname(self.yaml_path), image)
        self.image_path = image
        self.resolution = float(values['resolution'])
        self.origin = [float(item) for item in re.findall(
            r'[-+]?\d+(?:\.\d+)?', values['origin'])[:3]]
        self._load_pgm(image)

    def _load_pgm(self, image_path):
        with open(image_path, 'rb') as handle:
            magic = handle.readline().strip()
            if magic not in (b'P2', b'P5'):
                raise RuntimeError('Only PGM maps are supported: {}'.format(
                    image_path))
            tokens = []
            while len(tokens) < 3:
                line = handle.readline()
                if not line:
                    raise RuntimeError('Invalid PGM map: {}'.format(image_path))
                line = line.split(b'#', 1)[0]
                tokens.extend(line.split())
            self.width = int(tokens[0])
            self.height = int(tokens[1])
            max_value = int(tokens[2])
            if max_value > 255:
                raise RuntimeError('Only 8-bit PGM maps are supported: {}'.format(
                    image_path))
            if magic == b'P5':
                data = handle.read(self.width * self.height)
                if len(data) < self.width * self.height:
                    raise RuntimeError('Incomplete PGM map: {}'.format(image_path))
                self.pixels = bytearray(data[:self.width * self.height])
                return
            data_tokens = tokens[3:]
            for line in handle:
                line = line.split(b'#', 1)[0]
                data_tokens.extend(line.split())
            if len(data_tokens) < self.width * self.height:
                raise RuntimeError('Incomplete PGM map: {}'.format(image_path))
            self.pixels = bytearray(
                int(value) for value in data_tokens[:self.width * self.height])

    def _pixel_index(self, world_x, world_y):
        px = int((world_x - self.origin[0]) / self.resolution)
        py = self.height - 1 - int((world_y - self.origin[1]) / self.resolution)
        return px, py

    def _world_point(self, px, py):
        world_x = self.origin[0] + (px + 0.5) * self.resolution
        world_y = self.origin[1] + (self.height - 1 - py + 0.5) * self.resolution
        return world_x, world_y

    def _pixel_value(self, px, py):
        return self.pixels[py * self.width + px]

    def distance_to_occupied(self, world_x, world_y, search_radius=3.0):
        center_x, center_y = self._pixel_index(world_x, world_y)
        if (center_x < 0 or center_x >= self.width or
                center_y < 0 or center_y >= self.height):
            return 0.0
        radius_px = int(math.ceil(search_radius / self.resolution))
        best = None
        for py in range(max(0, center_y - radius_px),
                        min(self.height, center_y + radius_px + 1)):
            for px in range(max(0, center_x - radius_px),
                            min(self.width, center_x + radius_px + 1)):
                if self._pixel_value(px, py) > self.OCCUPIED_THRESHOLD:
                    continue
                occupied_x, occupied_y = self._world_point(px, py)
                distance = math.hypot(occupied_x - world_x, occupied_y - world_y)
                if best is None or distance < best:
                    best = distance
        if best is None:
            return float('inf')
        return best

    def is_safe(self, pose):
        distance = self.distance_to_occupied(
            pose.pose.position.x, pose.pose.position.y)
        return distance >= self.minimum_obstacle_distance, distance

    def _blocked_grid(self):
        if self._blocked_cache is not None:
            return self._blocked_cache

        blocked = bytearray(self.width * self.height)
        radius_px = int(math.ceil(
            self.path_minimum_obstacle_distance / self.resolution))
        occupied = []
        for py in range(self.height):
            for px in range(self.width):
                if self._pixel_value(px, py) <= self.OCCUPIED_THRESHOLD:
                    occupied.append((px, py))
        for occ_x, occ_y in occupied:
            for py in range(max(0, occ_y - radius_px),
                            min(self.height, occ_y + radius_px + 1)):
                for px in range(max(0, occ_x - radius_px),
                                min(self.width, occ_x + radius_px + 1)):
                    if math.hypot(px - occ_x, py - occ_y) <= radius_px:
                        blocked[py * self.width + px] = 1
        self._blocked_cache = blocked
        return blocked

    def _nearest_unblocked(self, px, py, blocked, max_radius_px):
        if (0 <= px < self.width and 0 <= py < self.height and
                not blocked[py * self.width + px]):
            return px, py
        best = None
        best_distance = None
        for radius in range(1, max_radius_px + 1):
            for y in range(max(0, py - radius), min(self.height, py + radius + 1)):
                for x in range(max(0, px - radius), min(self.width, px + radius + 1)):
                    if blocked[y * self.width + x]:
                        continue
                    distance = math.hypot(x - px, y - py)
                    if distance > radius:
                        continue
                    if best is None or distance < best_distance:
                        best = (x, y)
                        best_distance = distance
            if best is not None:
                return best
        return None

    def _path_forbidden_region(self, world_x, world_y):
        for region in self.forbidden_path_regions:
            if (region['min_x'] <= world_x <= region['max_x'] and
                    region['min_y'] <= world_y <= region['max_y']):
                return region
        return None

    def _grid_path_metrics(self, start, goal, blocked):
        previous = {start: None}
        distance_from_start = {start: 0.0}
        open_set = [(self._grid_heuristic(start, goal), 0.0, start)]
        neighbors = (
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)), (1, 1, math.sqrt(2.0)),
        )
        while open_set:
            _estimated_total, cost, current = heapq.heappop(open_set)
            if cost > distance_from_start.get(current, float('inf')):
                continue
            if current == goal:
                break
            for dx, dy, step_cost in neighbors:
                nx = current[0] + dx
                ny = current[1] + dy
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    continue
                item = (nx, ny)
                if blocked[ny * self.width + nx]:
                    continue
                if dx and dy:
                    if (blocked[current[1] * self.width + nx] or
                            blocked[ny * self.width + current[0]]):
                        continue
                next_cost = cost + step_cost * self.resolution
                if next_cost >= distance_from_start.get(item, float('inf')):
                    continue
                previous[item] = current
                distance_from_start[item] = next_cost
                priority = next_cost + self._grid_heuristic(item, goal)
                heapq.heappush(open_set, (priority, next_cost, item))

        if goal not in previous:
            return False, 0.0, None, {'reason': 'path_disconnected'}

        best = None
        forbidden = None
        current = goal
        while current is not None:
            world_x, world_y = self._world_point(current[0], current[1])
            region = self._path_forbidden_region(world_x, world_y)
            if region is not None:
                forbidden = region
            distance = self.distance_to_occupied(
                world_x, world_y,
                search_radius=max(self.minimum_obstacle_distance,
                                  self.path_minimum_obstacle_distance) +
                self.resolution)
            if math.isfinite(distance) and (best is None or distance < best):
                best = distance
            current = previous[current]
        if best is None:
            best = float('inf')
        if forbidden is not None:
            return False, best, distance_from_start[goal], {
                'reason': 'forbidden_path_region',
                'region': forbidden.get('name'),
            }
        return True, best, distance_from_start[goal], None

    def _grid_heuristic(self, item, goal):
        return math.hypot(item[0] - goal[0], item[1] - goal[1]) * self.resolution

    def path_metrics(self, start_pose, goal_pose):
        if start_pose is None:
            return True, None, None, None
        if start_pose.header.frame_id != goal_pose.header.frame_id:
            return True, None, None, None
        start_x = start_pose.pose.position.x
        start_y = start_pose.pose.position.y
        goal_x = goal_pose.pose.position.x
        goal_y = goal_pose.pose.position.y
        if math.hypot(goal_x - start_x, goal_y - start_y) <= 1e-6:
            return True, None, 0.0, None

        start_px, start_py = self._pixel_index(start_x, start_y)
        goal_px, goal_py = self._pixel_index(goal_x, goal_y)
        if (start_px < 0 or start_px >= self.width or
                start_py < 0 or start_py >= self.height or
                goal_px < 0 or goal_px >= self.width or
                goal_py < 0 or goal_py >= self.height):
            return False, 0.0, None, {'reason': 'path_endpoint_outside_map'}

        blocked = self._blocked_grid()
        search_radius_px = int(math.ceil(
            max(self.PATH_START_SKIP_M, self.path_minimum_obstacle_distance) /
            self.resolution))
        start = self._nearest_unblocked(
            start_px, start_py, blocked, search_radius_px)
        goal = self._nearest_unblocked(
            goal_px, goal_py, blocked, search_radius_px)
        if start is None or goal is None:
            return False, 0.0, None, {'reason': 'path_endpoint_blocked'}
        return self._grid_path_metrics(start, goal, blocked)

    def is_path_safe(self, start_pose, goal_pose):
        is_safe, distance, _path_length, _metadata = self.path_metrics(
            start_pose, goal_pose)
        return is_safe, distance


class TaskFlow(object):
    def __init__(self, args):
        self.args = args
        self.min_targets = args.min_targets
        self._map_safety = None
        if ((args.publish_goal or args.send_goal) and
                not args.disable_navigation_map_filter):
            self._map_safety = StaticMapSafety(
                args.navigation_map, args.navigation_min_obstacle_distance,
                args.navigation_path_min_obstacle_distance,
                forbidden_path_regions=indoor_route_forbidden_regions(args))
        if self.min_targets is None:
            if (args.publish_goal or args.send_goal or
                    target_classes_from_args(args) != ['bottle']):
                self.min_targets = 1
            else:
                self.min_targets = SCENARIOS[args.scenario]['expected_targets']
        self._event = threading.Event()
        self._lock = threading.Lock()
        self._latest = None
        self._latest_dropoff = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._subscriber = rospy.Subscriber(
            args.object_topic, GarbageObjectArray, self._callback, queue_size=10)

    def _objects_for_classes(self, message, class_names, min_confidence=None,
                             allow_behind=None):
        candidates = []
        class_names = set(class_names)
        if min_confidence is None:
            min_confidence = self.args.min_confidence
        if allow_behind is None:
            allow_behind = self.args.allow_behind
        for item in message.objects:
            if item.class_name not in class_names:
                continue
            if float(item.confidence) < min_confidence:
                continue
            point = item.position
            if not (finite(point.x) and finite(point.y) and finite(point.z)):
                continue
            if not allow_behind and point.x <= 0.0:
                continue
            planar_range = math.hypot(point.x, point.y)
            if planar_range <= 1e-6:
                continue
            candidates.append({
                'object': item,
                'range': planar_range,
            })
        return candidates

    def _candidate_objects(self, message):
        return self._objects_for_classes(
            message, target_classes_from_args(self.args))

    def _dropoff_objects(self, message):
        candidates = self._objects_for_classes(
            message, dropoff_classes_from_args(self.args),
            min_confidence=0.0, allow_behind=True)
        if not four_paper_ball_enabled(self.args):
            return candidates
        return self._dropoff_objects_in_four_paper_ball_region(message, candidates)

    def _object_navigation_pose(self, message, entry):
        if entry.get('cached_in_navigation_frame'):
            pose = PoseStamped()
            pose.header.frame_id = self.args.navigation_frame
            pose.header.stamp = ros_time_now_or_zero()
            pose.pose.position.x = float(entry['object'].position.x)
            pose.pose.position.y = float(entry['object'].position.y)
            pose.pose.position.z = float(entry['object'].position.z)
            pose.pose.orientation.w = 1.0
            return pose
        if not hasattr(message, 'header'):
            return None
        return self.transform_target_position(message, entry['object'])

    def _dropoff_objects_in_four_paper_ball_region(self, message, candidates):
        filtered = []
        for entry in candidates:
            try:
                target_pose = self._object_navigation_pose(message, entry)
            except RuntimeError:
                continue
            if target_pose is None:
                continue
            x, y = pose_position_xy(target_pose)
            if point_in_radius(x, y, FOUR_PAPER_BALL_DROPOFF_REGION):
                filtered.append(entry)
        return filtered

    def _entry_from_navigation_pose(self, source_entry, target_pose):
        copied = type(source_entry['object'])()
        copied.class_name = source_entry['object'].class_name
        copied.confidence = source_entry['object'].confidence
        copied.position = Point()
        copied.position.x = target_pose.pose.position.x
        copied.position.y = target_pose.pose.position.y
        copied.position.z = target_pose.pose.position.z
        copied.bbox_x = source_entry['object'].bbox_x
        copied.bbox_y = source_entry['object'].bbox_y
        copied.bbox_width = source_entry['object'].bbox_width
        copied.bbox_height = source_entry['object'].bbox_height
        entry = {
            'object': copied,
            'range': math.hypot(copied.position.x, copied.position.y),
            'cached_in_navigation_frame': True,
        }
        return entry

    def _cached_dropoff_valid_for_four_paper_ball(self):
        latest_dropoff = getattr(self, '_latest_dropoff', None)
        if latest_dropoff is None:
            return False
        if not latest_dropoff.get('cached_in_navigation_frame'):
            return False
        obj = latest_dropoff.get('object')
        if obj is None:
            return False
        return point_in_radius(
            float(obj.position.x), float(obj.position.y),
            FOUR_PAPER_BALL_DROPOFF_REGION)

    def remember_dropoff_message(self, message):
        candidates = self._dropoff_objects(message)
        if not candidates:
            if (four_paper_ball_enabled(self.args) and
                    not self._cached_dropoff_valid_for_four_paper_ball()):
                self._latest_dropoff = None
            return
        if not hasattr(message, 'header'):
            return
        source_entry = min(candidates, key=lambda entry: entry['range'])
        try:
            target_pose = self._object_navigation_pose(message, source_entry)
        except RuntimeError:
            return
        if target_pose is None:
            return
        self._latest_dropoff = self._entry_from_navigation_pose(
            source_entry, target_pose)

    def select_dropoff_target(self, message):
        candidates = self._dropoff_objects(message)
        if candidates:
            # Keep the latest visual trash-bin observation, but navigate to the
            # remembered map-frame drop-off point.  A stale camera/base-relative
            # candidate can project onto an occupied cell after the robot moves.
            if four_paper_ball_enabled(self.args):
                self._latest_dropoff = None
            self.remember_dropoff_message(message)
        elif (four_paper_ball_enabled(self.args) and
              not self._cached_dropoff_valid_for_four_paper_ball()):
            self._latest_dropoff = None
        latest_dropoff = getattr(self, '_latest_dropoff', None)
        if latest_dropoff is not None:
            return latest_dropoff
        if candidates:
            return min(candidates, key=lambda entry: entry['range'])
        raise RuntimeError(
            'No localized trash-bin/drop-off object from class(es) {} on {}'.format(
                dropoff_class_label(self.args), self.args.object_topic))

    def select_dropoff_navigation_targets(self, message):
        entry = self.select_dropoff_target(message)
        start_pose = None
        if (self._map_safety is not None and
                not getattr(self.args, 'disable_navigation_path_filter', False)):
            try:
                start_pose = self.current_navigation_pose()
            except RuntimeError as error:
                raise RuntimeError(
                    'Could not check drop-off navigation path safety: {}'.format(error))
        candidate_entries = []
        rejected = []
        for navigation_entry in self._navigation_entries(
                message, entry, start_pose=start_pose,
                approach_distance=self.args.dropoff_approach_distance,
                allow_distance_fallbacks=True):
            (pose, navigation_pose, target_pose, summary, is_safe, distance,
             path_distance, _path_length) = self._unpack_navigation_entry(
                 navigation_entry)
            metadata = self._navigation_candidate_metadata(
                entry, pose, navigation_pose, distance, path_distance, summary)
            if not is_safe:
                rejected.append(metadata)
                continue
            summary['target']['role'] = 'dropoff'
            summary['approach']['distance_m'] = self.args.dropoff_approach_distance
            candidate_entries.append({
                'pose': pose,
                'navigation_pose': navigation_pose,
                'target_pose': target_pose,
                'summary': summary,
            })
        if not candidate_entries:
            raise RuntimeError(
                'No trash-bin/drop-off navigation candidate satisfies {:.2f} m endpoint/path map safety'.format(
                    self.args.navigation_min_obstacle_distance))
        candidate_entries.sort(key=self._navigation_candidate_score)
        if rejected:
            candidate_entries[0]['summary']['approach']['rejected_navigation_candidates'] = rejected
        return candidate_entries

    def select_dropoff_navigation_target(self, message):
        return self.select_dropoff_navigation_targets(message)[0]

    def confirm_dropoff_before_pickup(self, message=None):
        if not dropoff_enabled_from_args(self.args):
            return None
        if message is not None:
            try:
                select_dropoff_navigation_targets_compat(self, message)
            except RuntimeError:
                pass
            else:
                return {'status': 'DROPOFF_CONFIRMED',
                        'source': 'current_perception'}
        latest_dropoff = getattr(self, '_latest_dropoff', None)
        if latest_dropoff is not None:
            if (not four_paper_ball_enabled(self.args) or
                    self._cached_dropoff_valid_for_four_paper_ball()):
                return {'status': 'DROPOFF_CONFIRMED',
                        'source': 'cached_navigation_frame_dropoff'}
            self._latest_dropoff = None
        timeout = getattr(self.args, 'dropoff_confirm_timeout', 8.0)
        if timeout <= 0.0:
            raise RuntimeError(
                'No pre-confirmed localized trash-bin/drop-off object from class(es) {} on {}'.format(
                    dropoff_class_label(self.args), self.args.object_topic))
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            latest_dropoff = getattr(self, '_latest_dropoff', None)
            if latest_dropoff is not None:
                if (not four_paper_ball_enabled(self.args) or
                        self._cached_dropoff_valid_for_four_paper_ball()):
                    return {'status': 'DROPOFF_CONFIRMED',
                            'source': 'cached_navigation_frame_dropoff'}
                self._latest_dropoff = None
            time.sleep(0.1)
        raise RuntimeError(
            'Timed out pre-confirming localized trash-bin/drop-off object from class(es) {} on {}'.format(
                dropoff_class_label(self.args), self.args.object_topic))

    def _callback(self, message):
        self.remember_dropoff_message(message)
        candidates = self._candidate_objects(message)
        if len(candidates) < self.min_targets:
            return
        with self._lock:
            self._latest = (message, candidates)
        self._event.set()

    def _wait_for_target_once(self, timeout, require_fresh=False):
        deadline = time.monotonic() + timeout
        if require_fresh:
            self._event.clear()
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self._event.wait(0.1):
                with self._lock:
                    message, candidates = self._latest
                return message, candidates
        raise RuntimeError(
            'Timed out waiting for at least {} {} object(s) on {}'.format(
                self.min_targets, target_class_label(self.args),
                self.args.object_topic))

    def current_navigation_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.args.navigation_frame, 'base_link', rospy.Time(0),
                rospy.Duration(self.args.navigation_transform_timeout))
        except tf2_ros.TransformException as error:
            raise RuntimeError(
                'Could not read current robot pose in {}: {}'.format(
                    self.args.navigation_frame, error))
        pose = PoseStamped()
        pose.header.frame_id = self.args.navigation_frame
        pose.header.stamp = ros_time_now_or_zero()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = 0.0
        pose.pose.orientation = transform.transform.rotation
        return pose

    def send_initial_scan_goal(self, yaw_offset):
        scan_pose = rotated_navigation_pose(
            self.current_navigation_pose(), yaw_offset)
        send_move_base_goal(self.args.move_base_action, scan_pose,
                            self.args.navigation_timeout)
        return scan_pose

    def wait_for_target(self, require_fresh=False):
        try:
            return self._wait_for_target_once(self.args.timeout, require_fresh)
        except RuntimeError:
            if (require_fresh or self.args.disable_initial_scan or
                    not self.args.send_goal):
                raise
        last_error = None
        for yaw_offset in (45.0, -45.0, 90.0, -90.0, 180.0):
            try:
                self.send_initial_scan_goal(math.radians(yaw_offset))
                return self._wait_for_target_once(
                    self.args.initial_scan_timeout, require_fresh=True)
            except RuntimeError as error:
                last_error = error
        if last_error is not None:
            raise last_error
        raise RuntimeError(
            'Timed out waiting for at least {} {} object(s) on {}'.format(
                self.min_targets, target_class_label(self.args),
                self.args.object_topic))

    def wait_for_target_with_timeout(self, timeout, require_fresh=False):
        try:
            return self._wait_for_target_once(timeout, require_fresh=require_fresh)
        except RuntimeError:
            raise

    def select_target(self, candidates):
        if self.args.selector == 'confidence':
            return max(candidates,
                       key=lambda entry: entry['object'].confidence)['object']
        return min(candidates, key=lambda entry: entry['range'])['object']

    def compute_goal(self, message, target):
        return self.compute_goal_with_offset(message, target, 0.0, legacy=True)

    def compute_goal_with_offset(self, message, target, angle_offset, legacy=False,
                                 approach_distance=None):
        if approach_distance is None:
            approach_distance = self.args.approach_distance
        target_x = float(target.position.x)
        target_y = float(target.position.y)
        target_z = float(target.position.z)
        target_range = math.hypot(target_x, target_y)
        target_bearing = math.atan2(target_y, target_x)

        if legacy:
            unit_x = target_x / target_range
            unit_y = target_y / target_range
            goal_range = max(0.0, target_range - approach_distance)
            goal_x = unit_x * goal_range
            goal_y = unit_y * goal_range
            yaw = target_bearing
        else:
            approach_angle = target_bearing + angle_offset
            goal_x = target_x - math.cos(approach_angle) * approach_distance
            goal_y = target_y - math.sin(approach_angle) * approach_distance
            yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        quaternion = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = message.header.frame_id or 'base_link'
        pose.header.stamp = ros_time_now_or_zero()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quaternion['x']
        pose.pose.orientation.y = quaternion['y']
        pose.pose.orientation.z = quaternion['z']
        pose.pose.orientation.w = quaternion['w']

        summary = {
            'status': 'READY_FOR_NAVIGATION',
            'mode': 'dry_run',
            'target': {
                'class_name': target.class_name,
                'confidence': float(target.confidence),
                'position': {
                    'x': target_x,
                    'y': target_y,
                    'z': target_z,
                },
                'range_m': target_range,
                'bbox': [
                    float(target.bbox_x), float(target.bbox_y),
                    float(target.bbox_width), float(target.bbox_height),
                ],
            },
            'approach': {
                'distance_m': approach_distance,
                'already_within_approach_distance': (
                    target_range <= approach_distance),
                'goal': pose_to_dict(pose),
            },
            'next_state': 'READY_FOR_PICKUP_AFTER_NAVIGATION',
        }
        if not legacy:
            summary['approach']['candidate_angle_offset_deg'] = math.degrees(
                angle_offset)
        if abs(approach_distance - self.args.approach_distance) > 1e-6:
            summary['approach']['candidate_approach_distance_m'] = approach_distance
        return pose, summary

    def transform_target_position(self, message, target):
        pose = PoseStamped()
        pose.header.frame_id = message.header.frame_id or 'base_link'
        pose.header.stamp = ros_time_now_or_zero()
        pose.pose.position.x = float(target.position.x)
        pose.pose.position.y = float(target.position.y)
        pose.pose.position.z = float(target.position.z)
        pose.pose.orientation.w = 1.0
        return self.transform_navigation_goal(pose)

    def transform_navigation_goal(self, pose):
        target_frame = self.args.navigation_frame
        if not target_frame or pose.header.frame_id == target_frame:
            pose.header.stamp = ros_time_now_or_zero()
            return pose

        source_pose = PoseStamped()
        source_pose.header.frame_id = pose.header.frame_id
        source_pose.header.stamp = rospy.Time(0)
        source_pose.pose = pose.pose
        try:
            navigation_pose = self._tf_buffer.transform(
                source_pose, target_frame,
                rospy.Duration(self.args.navigation_transform_timeout))
        except tf2_ros.TransformException as error:
            raise RuntimeError(
                'Could not transform navigation goal from {} to {}: {}'.format(
                    pose.header.frame_id, target_frame, error))
        navigation_pose.header.stamp = ros_time_now_or_zero()
        return navigation_pose

    def navigation_safety(self, pose, start_pose=None):
        if self._map_safety is None or pose.header.frame_id != 'map':
            return True, None, None, None, None
        is_safe, distance = self._map_safety.is_safe(pose)
        path_distance = None
        path_length = None
        path_metadata = None
        if (is_safe and start_pose is not None and
                not getattr(self.args, 'disable_navigation_path_filter', False)):
            path_result = self._map_safety.path_metrics(start_pose, pose)
            if len(path_result) == 4:
                is_safe, path_distance, path_length, path_metadata = path_result
            else:
                is_safe, path_distance, path_length = path_result
        return is_safe, distance, path_distance, path_length, path_metadata

    def _ordered_candidates(self, candidates):
        if self.args.selector == 'confidence':
            return sorted(
                candidates,
                key=lambda entry: entry['object'].confidence,
                reverse=True)
        return sorted(candidates, key=lambda entry: entry['range'])

    def _cached_navigation_frame_entry(self, entry, angle_offset=0.0, legacy=True,
                                       approach_distance=None, start_pose=None):
        if approach_distance is None:
            approach_distance = self.args.approach_distance
        target_x = float(entry['object'].position.x)
        target_y = float(entry['object'].position.y)
        target_z = float(entry['object'].position.z)
        if start_pose is not None and start_pose.header.frame_id == self.args.navigation_frame:
            reference_x = float(start_pose.pose.position.x)
            reference_y = float(start_pose.pose.position.y)
        else:
            current_pose = self.current_navigation_pose()
            reference_x = float(current_pose.pose.position.x)
            reference_y = float(current_pose.pose.position.y)
        vector_x = target_x - reference_x
        vector_y = target_y - reference_y
        target_range = math.hypot(vector_x, vector_y)
        if target_range <= 1e-6:
            vector_x = 1.0
            vector_y = 0.0
            target_range = 1.0
        target_bearing = math.atan2(vector_y, vector_x)
        if legacy:
            approach_angle = target_bearing
        else:
            approach_angle = target_bearing + angle_offset
        goal_x = target_x - math.cos(approach_angle) * approach_distance
        goal_y = target_y - math.sin(approach_angle) * approach_distance
        yaw = math.atan2(target_y - goal_y, target_x - goal_x)
        quaternion = yaw_to_quaternion(yaw)

        pose = PoseStamped()
        pose.header.frame_id = self.args.navigation_frame
        pose.header.stamp = ros_time_now_or_zero()
        pose.pose.position.x = goal_x
        pose.pose.position.y = goal_y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = quaternion['x']
        pose.pose.orientation.y = quaternion['y']
        pose.pose.orientation.z = quaternion['z']
        pose.pose.orientation.w = quaternion['w']

        target_pose = PoseStamped()
        target_pose.header.frame_id = self.args.navigation_frame
        target_pose.header.stamp = ros_time_now_or_zero()
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = target_z
        target_pose.pose.orientation.w = 1.0

        summary = {
            'status': 'READY_FOR_NAVIGATION',
            'mode': 'dry_run',
            'target': {
                'class_name': entry['object'].class_name,
                'confidence': float(entry['object'].confidence),
                'position': {
                    'x': target_x,
                    'y': target_y,
                    'z': target_z,
                },
                'range_m': target_range,
                'bbox': [
                    float(entry['object'].bbox_x), float(entry['object'].bbox_y),
                    float(entry['object'].bbox_width),
                    float(entry['object'].bbox_height),
                ],
                'source': 'cached_navigation_frame_dropoff',
            },
            'approach': {
                'distance_m': approach_distance,
                'already_within_approach_distance': (
                    target_range <= approach_distance),
                'goal': pose_to_dict(pose),
            },
            'next_state': 'READY_FOR_PICKUP_AFTER_NAVIGATION',
        }
        if not legacy:
            summary['approach']['candidate_angle_offset_deg'] = math.degrees(
                angle_offset)
        if abs(approach_distance - self.args.approach_distance) > 1e-6:
            summary['approach']['candidate_approach_distance_m'] = approach_distance
        return pose, pose, target_pose, summary

    def _navigation_entry(self, message, entry, angle_offset=0.0, legacy=True,
                          approach_distance=None, start_pose=None):
        if entry.get('cached_in_navigation_frame'):
            pose, navigation_pose, target_pose, summary = (
                self._cached_navigation_frame_entry(
                    entry, angle_offset=angle_offset, legacy=legacy,
                    approach_distance=approach_distance, start_pose=start_pose))
        else:
            pose, summary = self.compute_goal_with_offset(
                message, entry['object'], angle_offset, legacy=legacy,
                approach_distance=approach_distance)
            navigation_pose = self.transform_navigation_goal(pose)
            target_pose = self.transform_target_position(message, entry['object'])
        (is_safe, distance, path_distance, path_length, path_metadata) = (
            self.navigation_safety(navigation_pose, start_pose=start_pose))
        summary['target']['navigation_position'] = {
            'frame_id': target_pose.header.frame_id,
            'x': target_pose.pose.position.x,
            'y': target_pose.pose.position.y,
            'z': target_pose.pose.position.z,
        }
        summary['approach']['navigation_goal'] = pose_to_dict(navigation_pose)
        if distance is not None:
            summary['approach']['nearest_map_obstacle_distance_m'] = distance
        if path_distance is not None:
            summary['approach']['path_nearest_map_obstacle_distance_m'] = (
                path_distance)
        if path_length is not None:
            summary['approach']['routed_path_length_m'] = path_length
        if path_metadata is not None:
            summary['approach']['path_filter'] = path_metadata
        return (pose, navigation_pose, target_pose, summary, is_safe, distance,
                path_distance, path_length)

    def _candidate_approach_distances(self, target_range, preferred_distance=None):
        distances = [self.args.approach_distance
                     if preferred_distance is None else preferred_distance]
        if self._map_safety is not None:
            for distance in (0.75, 0.85, 0.95, 1.0, 1.05, 1.1, 1.2,
                             1.3, 1.4, 1.5, 1.6, 0.5):
                if distance < target_range and all(
                        abs(distance - item) > 1e-6 for item in distances):
                    distances.append(distance)
        return distances

    def _navigation_entries(self, message, entry, start_pose=None,
                            approach_distance=None, allow_distance_fallbacks=False):
        target_range = entry['range']
        angles = [0.0]
        if self._map_safety is not None:
            angles.extend([-15.0, 15.0, -25.0, 25.0, -35.0, 35.0,
                           -45.0, 45.0, -55.0, 55.0, -70.0, 70.0,
                           -90.0, 90.0, -110.0, 110.0, -135.0, 135.0,
                           -160.0, 160.0, 180.0])
        if approach_distance is None or allow_distance_fallbacks:
            distances = self._candidate_approach_distances(
                target_range, preferred_distance=approach_distance)
        else:
            distances = [approach_distance]
        for distance in distances:
            for index, degrees in enumerate(angles):
                legacy = (index == 0 and
                          abs(distance - self.args.approach_distance) <= 1e-6)
                yield self._navigation_entry(
                    message, entry, math.radians(degrees), legacy=legacy,
                    approach_distance=distance, start_pose=start_pose)

    def _is_duplicate_target(self, navigation_pose, selected):
        threshold = self.args.target_dedup_distance
        if threshold <= 0.0:
            return False
        x = navigation_pose.pose.position.x
        y = navigation_pose.pose.position.y
        for item in selected:
            other = item['navigation_pose']
            if navigation_pose.header.frame_id != other.header.frame_id:
                continue
            distance = math.hypot(
                x - other.pose.position.x,
                y - other.pose.position.y)
            if distance < threshold:
                return True
        return False

    def _is_duplicate_object(self, target_pose, selected):
        threshold = self.args.target_dedup_distance
        if threshold <= 0.0:
            return False
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
        for item in selected:
            other = item.get('target_pose')
            if other is None or target_pose.header.frame_id != other.header.frame_id:
                continue
            distance = math.hypot(
                x - other.pose.position.x,
                y - other.pose.position.y)
            if distance < threshold:
                return True
        return False

    def _navigation_candidate_metadata(self, entry, pose, navigation_pose,
                                       distance, path_distance, summary):
        approach = summary.get('approach', {})
        return {
            'class_name': entry['object'].class_name,
            'confidence': float(entry['object'].confidence),
            'range_m': entry['range'],
            'goal': pose_to_dict(pose),
            'navigation_goal': pose_to_dict(navigation_pose),
            'nearest_map_obstacle_distance_m': distance,
            'path_nearest_map_obstacle_distance_m': path_distance,
            'candidate_angle_offset_deg': approach.get(
                'candidate_angle_offset_deg', 0.0),
            'candidate_approach_distance_m': approach.get(
                'candidate_approach_distance_m',
                approach.get('distance_m')),
            'routed_path_length_m': approach.get('routed_path_length_m'),
            'path_filter': approach.get('path_filter'),
        }

    def _navigation_candidate_score(self, candidate):
        summary = candidate['summary']
        approach = summary.get('approach', {})
        target = summary.get('target', {}).get('navigation_position')
        goal = approach.get('navigation_goal', {}).get('position')
        approach_distance = approach.get(
            'candidate_approach_distance_m', approach.get('distance_m'))
        path_length = approach.get('routed_path_length_m')
        endpoint_clearance = approach.get('nearest_map_obstacle_distance_m')
        path_clearance = approach.get('path_nearest_map_obstacle_distance_m')
        angle_offset = abs(approach.get('candidate_angle_offset_deg', 0.0))
        direct_distance = None
        if target is not None and goal is not None:
            direct_distance = math.hypot(
                goal['x'] - target['x'], goal['y'] - target['y'])
        if path_length is None:
            path_length = direct_distance if direct_distance is not None else 0.0
        if endpoint_clearance is None:
            endpoint_clearance = self.args.navigation_min_obstacle_distance
        if path_clearance is None:
            path_clearance = getattr(
                self.args, 'navigation_path_min_obstacle_distance',
                self.args.navigation_min_obstacle_distance)

        # Use a weighted cost instead of lexicographic sorting.  The previous
        # version could prefer a perfect pickup distance even when it required a
        # much longer route; for competition runs, a short stable route with enough
        # clearance is usually better than a marginally closer stop pose.
        preferred_distance = self.args.approach_distance
        if approach_distance is None:
            approach_distance = preferred_distance
        approach_error = abs(approach_distance - preferred_distance)
        long_stop_penalty = max(0.0, approach_distance - 1.05)
        endpoint_margin = max(0.0, endpoint_clearance - self.args.navigation_min_obstacle_distance)
        path_minimum = getattr(
            self.args, 'navigation_path_min_obstacle_distance',
            self.args.navigation_min_obstacle_distance)
        path_margin = max(0.0, path_clearance - path_minimum)
        route_penalty = min(path_length, 8.0) * 0.35
        if path_length > 4.0:
            route_penalty += (path_length - 4.0) * 0.90
        clearance_bonus = min(endpoint_margin, 0.35) * 0.45 + min(path_margin, 0.35) * 0.35
        score = (
            approach_error * 1.8 +
            long_stop_penalty * 2.5 +
            route_penalty +
            angle_offset / 180.0 * 0.15 -
            clearance_bonus)
        approach['candidate_score'] = score
        approach['candidate_score_terms'] = {
            'approach_error_m': approach_error,
            'long_stop_penalty_m': long_stop_penalty,
            'route_penalty': route_penalty,
            'endpoint_margin_m': endpoint_margin,
            'path_margin_m': path_margin,
            'clearance_bonus': clearance_bonus,
            'angle_offset_deg': angle_offset,
        }
        return score


    def _unpack_navigation_entry(self, navigation_entry):
        if len(navigation_entry) == 8:
            return navigation_entry
        if len(navigation_entry) == 7:
            return navigation_entry + (None,)
        raise RuntimeError(
            'Unexpected navigation candidate tuple size: {}'.format(
                len(navigation_entry)))

    def select_navigation_targets(self, message, candidates, count, existing=None):
        selected = []
        existing = list(existing or [])
        rejected = []
        duplicates = []
        start_pose = None
        if (self._map_safety is not None and
                not getattr(self.args, 'disable_navigation_path_filter', False)):
            try:
                start_pose = self.current_navigation_pose()
            except RuntimeError as error:
                raise RuntimeError(
                    'Could not check navigation path safety: {}'.format(error))
        for entry in self._ordered_candidates(candidates):
            candidate_entries = []
            duplicate_entry = False
            for navigation_entry in self._navigation_entries(
                    message, entry, start_pose=start_pose):
                (pose, navigation_pose, target_pose, summary, is_safe, distance,
                 path_distance, _path_length) = self._unpack_navigation_entry(
                     navigation_entry)
                metadata = self._navigation_candidate_metadata(
                    entry, pose, navigation_pose, distance, path_distance, summary)
                if self._is_duplicate_object(target_pose, existing + selected):
                    duplicates.append(metadata)
                    duplicate_entry = True
                    break
                if not is_safe:
                    rejected.append(metadata)
                    continue
                candidate_entries.append({
                    'pose': pose,
                    'navigation_pose': navigation_pose,
                    'target_pose': target_pose,
                    'summary': summary,
                })
            if duplicate_entry:
                continue
            candidate_entries.sort(key=self._navigation_candidate_score)
            for candidate in candidate_entries:
                if self._is_duplicate_target(
                        candidate['navigation_pose'], existing + selected):
                    duplicates.append(self._navigation_candidate_metadata(
                        entry, candidate['pose'], candidate['navigation_pose'],
                        candidate['summary']['approach'].get(
                            'nearest_map_obstacle_distance_m'),
                        candidate['summary']['approach'].get(
                            'path_nearest_map_obstacle_distance_m'),
                        candidate['summary']))
                    continue
                selected.append(candidate)
                break
            if len(selected) >= count:
                break
        if selected:
            metadata = {}
            if rejected:
                metadata['rejected_navigation_candidates'] = rejected
            if duplicates:
                metadata['duplicate_navigation_candidates'] = duplicates
            return selected, metadata
        raise RuntimeError(
            'No navigation candidate satisfies {:.2f} m endpoint/path map safety'.format(
                self.args.navigation_min_obstacle_distance))

    def select_navigation_target(self, message, candidates):
        selected, metadata = self.select_navigation_targets(message, candidates, 1)
        summary = selected[0]['summary']
        for key, value in metadata.items():
            summary['approach'][key] = value
        return selected[0]['pose'], selected[0]['navigation_pose'], summary


def publish_goal(pose):
    publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped,
                                queue_size=1, latch=True)
    deadline = time.monotonic() + 2.0
    while publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        time.sleep(0.05)
    publisher.publish(pose)
    time.sleep(0.5)


def send_move_base_goal(action_name, pose, timeout):
    import actionlib
    from actionlib_msgs.msg import GoalStatus
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

    client = actionlib.SimpleActionClient(action_name, MoveBaseAction)
    if not client.wait_for_server(rospy.Duration(10.0)):
        raise RuntimeError('move_base action server {} is not available'.format(
            action_name))
    goal = MoveBaseGoal()
    goal.target_pose = pose
    client.send_goal(goal)
    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        client.cancel_goal()
        raise RuntimeError('move_base did not finish within {:.1f}s'.format(timeout))
    state = client.get_state()
    if state != GoalStatus.SUCCEEDED:
        raise RuntimeError('move_base finished with state {}'.format(state))


def clear_move_base_costmaps(service_name='/move_base/clear_costmaps', timeout=5.0):
    from std_srvs.srv import Empty

    rospy.wait_for_service(service_name, timeout=timeout)
    service = rospy.ServiceProxy(service_name, Empty)
    service()


def send_final_exit_goal_with_retry(args, pose, timeout):
    attempts = []
    max_attempts = max(1, int(getattr(args, 'final_exit_attempts', 4)))
    last_error = None
    for attempt_index in range(1, max_attempts + 1):
        recovery = None
        if attempt_index > 1:
            recovery = 'clear_costmaps'
            try:
                clear_move_base_costmaps()
                time.sleep(1.0)
            except (rospy.ROSException, rospy.ServiceException) as recovery_error:
                attempts.append({
                    'attempt': attempt_index,
                    'status': 'EXIT_FAILED',
                    'recovery': recovery,
                    'error': str(recovery_error),
                })
                last_error = recovery_error
                continue
        try:
            send_move_base_goal(args.move_base_action, pose, timeout)
            summary = {
                'attempt': attempt_index,
                'status': 'EXIT_REACHED',
            }
            if recovery:
                summary['recovery'] = recovery
            attempts.append(summary)
            return attempts
        except RuntimeError as goal_error:
            summary = {
                'attempt': attempt_index,
                'status': 'EXIT_FAILED',
                'error': str(goal_error),
            }
            if recovery:
                summary['recovery'] = recovery
            attempts.append(summary)
            last_error = goal_error
    raise RuntimeError('final exit failed after {} same-goal attempts: {}'.format(
        max_attempts, last_error))


def ros_time_now_or_zero():
    try:
        return rospy.Time.now()
    except rospy.exceptions.ROSInitException:
        return rospy.Time(0)

def quaternion_to_yaw(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z +
                       orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y +
                             orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotated_navigation_pose(pose, yaw_offset):
    target = PoseStamped()
    target.header.frame_id = pose.header.frame_id
    target.header.stamp = ros_time_now_or_zero()
    target.pose.position.x = pose.pose.position.x
    target.pose.position.y = pose.pose.position.y
    target.pose.position.z = pose.pose.position.z
    yaw = quaternion_to_yaw(pose.pose.orientation) + yaw_offset
    quaternion = yaw_to_quaternion(yaw)
    target.pose.orientation.x = quaternion['x']
    target.pose.orientation.y = quaternion['y']
    target.pose.orientation.z = quaternion['z']
    target.pose.orientation.w = quaternion['w']
    return target


def send_recovery_scan_goal(args, pose, yaw_offset):
    scan_pose = rotated_navigation_pose(pose, yaw_offset)
    send_move_base_goal(args.move_base_action, scan_pose,
                        args.navigation_timeout)
    return scan_pose


def indoor_route_forbidden_regions(args):
    if getattr(args, 'disable_indoor_route_filter', False):
        return []
    if getattr(args, 'scenario', None) != 'four_paper_ball':
        return []
    # Keep normal task navigation inside the competition rooms.  The start opening
    # is the bottom living-room entrance (x=6.30..7.20); routes may approach the
    # doorway from inside, but should not step into the outside strip and re-enter.
    return [{
        'name': 'outside_bottom_living_room_entrance',
        'min_x': 5.80,
        'max_x': 7.70,
        'min_y': -1.20,
        'max_y': 0.15,
    }]


def room_search_waypoints(args):
    spec = getattr(args, 'search_waypoints', 'auto')
    if not spec or spec.lower() in ('none', 'off', 'false', '0'):
        return []
    if spec.lower() == 'auto':
        if args.scenario not in ('four_occlusion', 'four_paper_ball'):
            return []
        return [
            {'name': 'living_room_center', 'x': 6.75, 'y': 2.20, 'yaw': math.radians(90.0)},
            {'name': 'bedroom_center', 'x': 1.90, 'y': 2.90, 'yaw': math.radians(-90.0)},
            {'name': 'dining_room_center', 'x': 2.20, 'y': 5.75, 'yaw': math.radians(-45.0)},
            {'name': 'kitchen_center', 'x': 6.80, 'y': 6.25, 'yaw': math.radians(-90.0)},
        ]

    waypoints = []
    for index, raw_item in enumerate(spec.split(','), 1):
        item = raw_item.strip()
        if not item:
            continue
        parts = item.split(':')
        if len(parts) != 4:
            raise RuntimeError(
                'Invalid --search-waypoints item {!r}; expected name:x:y:yaw_deg'.format(
                    item))
        name, x, y, yaw_degrees = parts
        waypoints.append({
            'name': name,
            'x': float(x),
            'y': float(y),
            'yaw': math.radians(float(yaw_degrees)),
        })
    return waypoints


def room_patrol_enabled(args):
    spec = str(getattr(args, 'room_patrol', 'auto')).lower()
    if spec in ('none', 'off', 'false', '0', 'disabled'):
        return False
    if spec in ('on', 'true', '1', 'enabled'):
        return True
    if spec != 'auto':
        raise RuntimeError(
            'Invalid --room-patrol {!r}; expected auto, on, or off'.format(
                getattr(args, 'room_patrol', 'auto')))
    return (getattr(args, 'scenario', None) == 'four_paper_ball' and
            getattr(args, 'send_goal', False) and
            getattr(args, 'visit_count', 1) > 1)


def room_patrol_waypoints(args):
    if not room_patrol_enabled(args):
        return []
    if getattr(args, 'scenario', None) != 'four_paper_ball':
        return []
    # User-selected WaterPlus/RViz waypoints captured on 2026-08-17 for the
    # competition-style scene.  WP1..WP5 are patrol/scan points; WP6 is handled
    # by final_exit_waypoint().  Keep this at the task layer so shared navigation
    # configs remain untouched.
    return [
        {
            'name': 'wp1_living_room_left_180_scan',
            'room': 'living_room',
            'x': 8.39526,
            'y': 1.52033,
            'yaw': 0.06993948015094072,
            'scan_offsets': [math.radians(value) for value in (45.0, 90.0, 135.0, 180.0)],
            'classes': ['bottle'],
        },
        {
            'name': 'wp2_kitchen_260_scan',
            'room': 'kitchen',
            'x': 7.02370,
            'y': 6.98012,
            'yaw': 1.5059047125371625,
            'scan_offsets': [math.radians(value) for value in (65.0, 130.0, 195.0, 260.0)],
            'classes': ['bottle'],
        },
        {
            'name': 'wp3_bedroom_front_check',
            'room': 'bedroom',
            'x': 2.80257,
            'y': 4.03038,
            'yaw': -1.5575892800701567,
            # Front-facing check only; if the bedside paper ball is not marked
            # from this doorway pose, WP4 provides the fallback without touching it.
            'scan_offsets': [],
            'classes': ['paper_ball'],
        },
        {
            'name': 'wp4_bedroom_front_backup',
            'room': 'bedroom',
            'x': 1.02032,
            'y': 4.07878,
            'yaw': -1.5652524188585184,
            'scan_offsets': [],
            'classes': ['paper_ball'],
        },
        {
            'name': 'wp5_dining_room_360_scan',
            'room': 'dining_room',
            'x': 2.32366,
            'y': 6.20868,
            'yaw': 1.5521172892034156,
            'scan_offsets': [math.radians(value) for value in (90.0, 180.0, -90.0, 0.0)],
            'classes': ['box'],
        },
    ]


def room_patrol_target_classes(waypoint):
    return set(waypoint.get('classes') or [])


def room_patrol_candidate_in_room(flow, message, entry, waypoint):
    if not four_paper_ball_enabled(flow.args):
        return True
    region = FOUR_PAPER_BALL_ROOM_REGIONS.get(waypoint.get('room'))
    if region is None:
        return True
    try:
        target_pose = flow._object_navigation_pose(message, entry)
    except RuntimeError:
        return False
    if target_pose is None:
        return False
    x, y = pose_position_xy(target_pose)
    return point_in_rect(x, y, region)


def room_patrol_candidate_filter_details(flow, message, waypoint):
    classes = room_patrol_target_classes(waypoint)
    accepted = []
    class_rejected = []
    room_rejected = []
    transform_rejected = []
    for entry in flow._candidate_objects(message):
        class_name = entry['object'].class_name
        if classes and class_name not in classes:
            class_rejected.append(entry)
            continue
        try:
            in_room = room_patrol_candidate_in_room(flow, message, entry, waypoint)
        except RuntimeError:
            transform_rejected.append(entry)
            continue
        if not in_room:
            room_rejected.append(entry)
            continue
        accepted.append(entry)
    return {
        'accepted': accepted,
        'class_rejected': class_rejected,
        'room_rejected': room_rejected,
        'transform_rejected': transform_rejected,
    }


def room_patrol_candidates(flow, message, waypoint):
    return room_patrol_candidate_filter_details(
        flow, message, waypoint)['accepted']


def room_patrol_filter_metadata(details):
    return {
        'candidate_count': len(details['accepted']),
        'class_rejected_count': len(details['class_rejected']),
        'room_rejected_count': len(details['room_rejected']),
        'transform_rejected_count': len(details['transform_rejected']),
    }


def wait_for_room_patrol_scan_target(args, flow, waypoint, timeout):
    deadline = time.monotonic() + timeout
    last_error = None
    sample_count = 0
    last_message = None
    last_candidates = []
    last_room_candidates = []
    last_details = {
        'accepted': [],
        'class_rejected': [],
        'room_rejected': [],
        'transform_rejected': [],
    }
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            break
        try:
            message, candidates = flow.wait_for_target_with_timeout(
                remaining, require_fresh=True)
        except RuntimeError as error:
            last_error = error
            break
        sample_count += 1
        details = room_patrol_candidate_filter_details(flow, message, waypoint)
        room_candidates = details['accepted']
        if len(room_candidates) >= len(last_room_candidates):
            last_message = message
            last_candidates = candidates
            last_room_candidates = room_candidates
            last_details = details
        if room_candidates:
            metadata = {
                'sample_count': sample_count,
            }
            metadata.update(room_patrol_filter_metadata(details))
            return message, candidates, room_candidates, metadata
    if last_message is not None:
        metadata = {
            'sample_count': sample_count,
        }
        metadata.update(room_patrol_filter_metadata(last_details))
        if last_error is not None:
            metadata['last_error'] = str(last_error)
        return last_message, last_candidates, last_room_candidates, metadata
    if last_error is not None:
        raise last_error
    raise RuntimeError('Timed out waiting for room-filtered {} detection for {}'.format(
        ','.join(waypoint.get('classes', [])), waypoint['room']))


def send_room_patrol_scan_goals(args, waypoint, origin_pose):
    scans = []
    for scan_index, yaw_offset in enumerate(waypoint.get('scan_offsets', []), 1):
        scan_summary = {
            'scan_index': scan_index,
            'yaw_offset_deg': math.degrees(yaw_offset),
        }
        scan_pose = send_recovery_scan_goal(args, origin_pose, yaw_offset)
        scan_summary['status'] = 'SCAN_GOAL_REACHED'
        scan_summary['pose'] = pose_to_dict(scan_pose)
        scans.append(scan_summary)
    return scans


def wait_for_room_patrol_target(args, flow, waypoint, room_summary, waypoint_pose):
    try:
        message, candidates = flow.wait_for_target_with_timeout(
            args.recovery_scan_timeout, require_fresh=True)
        room_candidates = room_patrol_candidates(flow, message, waypoint)
        if room_candidates:
            room_summary['detection_source'] = 'room_waypoint_direct'
            return message, candidates, room_candidates
        room_summary['initial_detection_error'] = (
            'fresh detections did not include {}'.format(
                ','.join(waypoint.get('classes', []))))
    except RuntimeError as first_error:
        room_summary['initial_detection_error'] = str(first_error)

    scans = []
    for scan_index, yaw_offset in enumerate(waypoint.get('scan_offsets', []), 1):
        scan_summary = {
            'scan_index': scan_index,
            'yaw_offset_deg': math.degrees(yaw_offset),
        }
        try:
            scan_pose = send_recovery_scan_goal(args, waypoint_pose, yaw_offset)
            scan_summary['status'] = 'SCAN_GOAL_REACHED'
            scan_summary['pose'] = pose_to_dict(scan_pose)
            message, candidates, room_candidates, metadata = wait_for_room_patrol_scan_target(
                args, flow, waypoint, args.recovery_scan_timeout)
            scan_summary['sample_count'] = metadata.get('sample_count', 0)
            scan_summary['candidate_count'] = metadata.get(
                'candidate_count', len(room_candidates))
            scans.append(scan_summary)
            room_summary['scans'] = scans
            if room_candidates:
                room_summary['detection_source'] = 'room_patrol_scan'
                return message, candidates, room_candidates
        except RuntimeError as scan_error:
            scan_summary['status'] = 'SCAN_FAILED'
            scan_summary['error'] = str(scan_error)
            scans.append(scan_summary)
            room_summary['scans'] = scans
            continue

    raise RuntimeError('No {} target detected for {}'.format(
        ','.join(waypoint.get('classes', [])), waypoint['room']))


def make_navigation_pose(frame_id, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = ros_time_now_or_zero()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    quaternion = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = quaternion['x']
    pose.pose.orientation.y = quaternion['y']
    pose.pose.orientation.z = quaternion['z']
    pose.pose.orientation.w = quaternion['w']
    return pose


def send_search_waypoint_goal(args, waypoint):
    pose = make_navigation_pose(
        args.navigation_frame, waypoint['x'], waypoint['y'], waypoint['yaw'])
    timeout = min(args.navigation_timeout, args.search_waypoint_timeout)
    if timeout <= 0.0:
        timeout = args.navigation_timeout
    send_move_base_goal(args.move_base_action, pose, timeout)
    return pose


def final_exit_waypoint(args):
    spec = getattr(args, 'final_exit', 'auto')
    if not spec or spec.lower() in ('none', 'off', 'false', '0'):
        return None
    if spec.lower() == 'auto':
        if getattr(args, 'scenario', None) != 'four_paper_ball':
            return None
        # The robot starts at the bottom living-room entrance in four_rooms.world.
        # The required competition egress is therefore the other opening.  Use the
        # user-selected WaterPlus WP6 exit pose saved on 2026-08-17.
        return {'name': 'wp6_left_dining_room_exit',
                'x': 0.190553, 'y': 7.89226,
                'yaw': -3.084572395452059,
                'excluded_start_opening': 'bottom_living_room_entrance'}
    parts = spec.split(':')
    if len(parts) != 4:
        raise RuntimeError(
            'Invalid --final-exit {!r}; expected auto, none, or name:x:y:yaw_deg'.format(
                spec))
    name, x, y, yaw_degrees = parts
    return {'name': name, 'x': float(x), 'y': float(y),
            'yaw': math.radians(float(yaw_degrees))}


def navigate_to_final_exit(args):
    waypoint = final_exit_waypoint(args)
    if waypoint is None:
        return None
    pose = make_navigation_pose(
        args.navigation_frame, waypoint['x'], waypoint['y'], waypoint['yaw'])
    timeout = min(args.navigation_timeout, args.final_exit_timeout)
    if timeout <= 0.0:
        timeout = args.navigation_timeout
    summary = {
        'name': waypoint['name'],
        'x': waypoint['x'],
        'y': waypoint['y'],
        'yaw_deg': math.degrees(waypoint['yaw']),
        'pose': pose_to_dict(pose),
        'mode': 'move_base_action',
    }
    if 'excluded_start_opening' in waypoint:
        summary['excluded_start_opening'] = waypoint['excluded_start_opening']
    summary['attempts'] = send_final_exit_goal_with_retry(args, pose, timeout)
    summary['status'] = 'EXIT_REACHED'
    return summary


def navigation_candidate_route_length(item):
    return item.get('summary', {}).get('approach', {}).get(
        'routed_path_length_m')


def should_reposition_before_navigation(args, item):
    threshold = getattr(args, 'search_reposition_path_length', 4.0)
    if threshold <= 0.0:
        return False
    route_length = navigation_candidate_route_length(item)
    return route_length is not None and route_length > threshold


def ordered_waypoints_for_candidate(search_waypoints, item):
    target = item.get('summary', {}).get('target', {}).get(
        'navigation_position')
    if not target:
        return list(search_waypoints)
    return sorted(search_waypoints, key=lambda waypoint: math.hypot(
        waypoint['x'] - target['x'], waypoint['y'] - target['y']))


def _legacy_dropoff_navigation_target_selector(flow):
    selector = getattr(flow, 'select_dropoff_navigation_target', None)
    if selector is None:
        return None
    if 'select_dropoff_navigation_target' in getattr(flow, '__dict__', {}):
        return selector
    if not hasattr(flow, 'select_dropoff_navigation_targets'):
        return selector
    return None


def select_dropoff_navigation_targets_compat(flow, message):
    legacy_selector = _legacy_dropoff_navigation_target_selector(flow)
    try:
        return flow.select_dropoff_navigation_targets(message)
    except (AttributeError, TypeError, RuntimeError):
        if legacy_selector is None:
            raise
        return [legacy_selector(message)]


def confirm_dropoff_before_pickup(args, flow, message=None, summary=None):
    if not dropoff_enabled_from_args(args):
        return None
    try:
        result = flow.confirm_dropoff_before_pickup(message)
    except AttributeError:
        if message is None:
            raise RuntimeError(
                'No perception message available for pre-pickup trash-bin/drop-off confirmation')
        select_dropoff_navigation_targets_compat(flow, message)
        result = {'status': 'DROPOFF_CONFIRMED', 'source': 'current_perception'}
    if summary is not None and result is not None:
        summary['pre_pickup_dropoff'] = result
    return result


def _target_class_from_summary(summary):
    target = (summary or {}).get('target', {})
    return target.get('class_name') or target.get('name')


def run_pickup_action(args, target_summary, sequence_index):
    mode = getattr(args, 'pickup_action', 'dry-run')
    if mode == 'none':
        return None
    if mode != 'dry-run':
        raise RuntimeError('Unsupported pickup action mode: {}'.format(mode))
    summary = {
        'sequence_index': sequence_index,
        'action': 'pickup',
        'mode': 'dry_run',
        'status': 'PICKUP_DRY_RUN_COMPLETE',
        'hardware_commanded': False,
    }
    class_name = _target_class_from_summary(target_summary)
    if class_name:
        summary['target_class'] = class_name
    return summary


def run_dropoff_action(args, dropoff_summary, sequence_index):
    mode = getattr(args, 'dropoff_action', 'dry-run')
    if mode == 'none':
        return None
    if mode != 'dry-run':
        raise RuntimeError('Unsupported drop-off action mode: {}'.format(mode))
    summary = {
        'sequence_index': sequence_index,
        'action': 'dropoff',
        'mode': 'dry_run',
        'status': 'DROPOFF_DRY_RUN_COMPLETE',
        'hardware_commanded': False,
    }
    class_name = _target_class_from_summary(dropoff_summary)
    if class_name:
        summary['target_class'] = class_name
    return summary


def record_pickup_action(args, target_summary, sequence_index):
    action_summary = run_pickup_action(args, target_summary, sequence_index)
    if action_summary is not None:
        target_summary['pickup_action'] = action_summary
    return action_summary


def record_dropoff_action(args, dropoff_summary, sequence_index):
    action_summary = run_dropoff_action(args, dropoff_summary, sequence_index)
    if action_summary is not None:
        dropoff_summary['dropoff_action'] = action_summary
    return action_summary


def _summary_targets_for_audit(summary):
    if 'targets' in summary:
        return list(summary.get('targets') or [])
    if 'target' in summary:
        return [summary]
    return []


def _add_audit_check(checks, name, passed, required=True, details=None):
    if passed:
        status = 'OK'
    elif required:
        status = 'FAIL'
    else:
        status = 'SKIPPED'
    item = {'name': name, 'status': status}
    if details is not None:
        item['details'] = details
    checks.append(item)


def build_competition_flow_audit(args, summary):
    target_classes = target_classes_from_args(args)
    dropoff_classes = dropoff_classes_from_args(args)
    dropoff_enabled = dropoff_enabled_from_args(args)
    competition_required = competition_flow_required_from_args(args)
    targets = _summary_targets_for_audit(summary)
    reached_targets = [
        item for item in targets
        if item.get('status') in ('READY_FOR_PICKUP', 'DROPOFF_REACHED')
    ]
    requested_count = summary.get(
        'requested_targets', getattr(args, 'visit_count', len(targets)))
    visited_count = summary.get('visited_targets', len(reached_targets))
    complete_statuses = set([
        'SEQUENTIAL_NAVIGATION_COMPLETE',
        'ROOM_PATROL_COMPLETE',
        'READY_FOR_PICKUP',
        'DROPOFF_REACHED',
    ])
    completed = (summary.get('status') in complete_statuses and
                 visited_count >= requested_count)

    checks = []
    overlap = sorted(set(target_classes) & set(dropoff_classes))
    _add_audit_check(
        checks, 'pickup_targets_exclude_dropoff', not overlap, True,
        {'target_classes': target_classes,
         'dropoff_classes': dropoff_classes,
         'overlap': overlap})
    _add_audit_check(
        checks, 'dropoff_enabled_for_competition_flow', dropoff_enabled,
        competition_required,
        {'competition_flow_required': competition_required,
         'dropoff_enabled': dropoff_enabled})
    _add_audit_check(
        checks, 'requested_targets_completed', completed, True,
        {'requested_targets': requested_count,
         'visited_targets': visited_count,
         'summary_status': summary.get('status')})

    precheck_required = dropoff_enabled and bool(targets)
    precheck_ok = all(
        item.get('pre_pickup_dropoff', {}).get('status') == 'DROPOFF_CONFIRMED'
        for item in reached_targets) and bool(reached_targets or not precheck_required)
    _add_audit_check(
        checks, 'trash_bin_confirmed_before_each_pickup', precheck_ok,
        precheck_required,
        {'checked_targets': len(reached_targets),
         'required': precheck_required})

    dropoff_required = dropoff_enabled and bool(targets)
    dropoff_ok = all(
        item.get('status') == 'DROPOFF_REACHED' and
        isinstance(item.get('dropoff'), dict) and
        item['dropoff'].get('status') == 'DROPOFF_REACHED'
        for item in reached_targets) and bool(reached_targets or not dropoff_required)
    _add_audit_check(
        checks, 'dropoff_reached_after_each_pickup', dropoff_ok,
        dropoff_required,
        {'checked_targets': len(reached_targets),
         'required': dropoff_required})

    final_exit_required = final_exit_waypoint(args) is not None
    final_exit = summary.get('final_exit') or {}
    final_exit_ok = (not final_exit_required or
                     final_exit.get('status') == 'EXIT_REACHED')
    _add_audit_check(
        checks, 'final_exit_recorded_after_completed_dropoffs', final_exit_ok,
        final_exit_required,
        {'required': final_exit_required,
         'final_exit_status': final_exit.get('status')})

    pickup_expected = getattr(args, 'pickup_action', 'dry-run') == 'dry-run'
    dropoff_expected = (dropoff_enabled and
                        getattr(args, 'dropoff_action', 'dry-run') == 'dry-run')
    pickup_actions = [item.get('pickup_action') for item in reached_targets
                      if item.get('pickup_action') is not None]
    dropoff_actions = []
    for item in reached_targets:
        dropoff = item.get('dropoff')
        if isinstance(dropoff, dict) and dropoff.get('dropoff_action') is not None:
            dropoff_actions.append(dropoff.get('dropoff_action'))
        elif item.get('dropoff_action') is not None:
            dropoff_actions.append(item.get('dropoff_action'))
    action_summaries = pickup_actions + dropoff_actions
    pickup_presence_ok = (not pickup_expected or
                          len(pickup_actions) == len(reached_targets))
    dropoff_presence_ok = (not dropoff_expected or
                           len(dropoff_actions) == len(reached_targets))
    hardware_safe = all(
        item.get('hardware_commanded') is False for item in action_summaries)
    actions_ok = pickup_presence_ok and dropoff_presence_ok and hardware_safe
    _add_audit_check(
        checks, 'action_hooks_hardware_safe', actions_ok, bool(reached_targets),
        {'pickup_action_mode': getattr(args, 'pickup_action', 'dry-run'),
         'dropoff_action_mode': getattr(args, 'dropoff_action', 'dry-run'),
         'pickup_actions_recorded': len(pickup_actions),
         'dropoff_actions_recorded': len(dropoff_actions),
         'checked_targets': len(reached_targets)})

    if getattr(args, 'garbage_competition_mode', False):
        _add_audit_check(
            checks, 'garbage_competition_quota_completed',
            completed_dropoff_count(reached_targets) >= garbage_quota_from_args(args),
            True,
            {'garbage_quota': garbage_quota_from_args(args),
             'completed_dropoffs': completed_dropoff_count(reached_targets),
             'visited_targets': visited_count,
             'skipped_rooms_after_quota': summary.get('skipped_rooms_after_quota', [])})

    status = ('COMPETITION_AUDIT_PASSED' if
              all(item['status'] != 'FAIL' for item in checks) else
              'COMPETITION_AUDIT_FAILED')
    return {
        'status': status,
        'competition_flow_required': competition_required,
        'checks': checks,
    }


def navigate_to_dropoff(args, dropoff_items, sequence_index):
    if isinstance(dropoff_items, dict):
        dropoff_items = [dropoff_items]
    attempts = []
    last_error = None
    for attempt_index, dropoff_item in enumerate(dropoff_items, 1):
        dropoff_summary = dropoff_item['summary']
        dropoff_summary['sequence_index'] = sequence_index
        dropoff_summary['mode'] = 'move_base_action'
        dropoff_summary['dropoff_attempt_index'] = attempt_index
        try:
            send_move_base_goal(args.move_base_action, dropoff_item['navigation_pose'],
                                args.navigation_timeout)
        except RuntimeError as error:
            last_error = str(error)
            failed_summary = dict(dropoff_summary)
            failed_summary['status'] = 'DROPOFF_NAVIGATION_FAILED'
            failed_summary['error'] = last_error
            attempts.append(failed_summary)
            continue
        dropoff_summary['status'] = 'DROPOFF_REACHED'
        record_dropoff_action(args, dropoff_summary, sequence_index)
        dropoff_summary['next_state'] = 'READY_FOR_NEXT_GARBAGE_TARGET'
        if attempts:
            dropoff_summary['failed_dropoff_attempts'] = attempts
        return dropoff_summary
    raise RuntimeError(last_error or 'No drop-off navigation candidates were attempted')


def run_sequential_navigation(args, flow, message, candidates,
                              initial_search_waypoint_visits=None,
                              initial_perception_source='initial_perception'):
    results = []
    visited = []
    failed_goals = []
    queued = []
    filtering = []
    navigation_failures = []
    recovery_scans = []
    search_waypoint_visits = list(initial_search_waypoint_visits or [])
    search_waypoints = room_search_waypoints(args)
    search_waypoint_index = len(search_waypoint_visits)
    pre_navigation_repositioned_sequences = set()
    recovery_scan_offsets = [math.radians(value) for value in (
        45.0, -45.0, 90.0, -90.0, 135.0, -135.0, 180.0)]
    stop_reason = None
    attempts = 0
    max_attempts = max(args.visit_count * 8, args.visit_count + 3)

    try:
        queued, metadata = flow.select_navigation_targets(
            message, candidates, args.visit_count, existing=visited + failed_goals)
        if metadata:
            step_metadata = {'sequence_index': 1, 'source': initial_perception_source}
            step_metadata.update(metadata)
            filtering.append(step_metadata)
    except RuntimeError as error:
        raise

    while len(results) < args.visit_count:
        if attempts >= max_attempts:
            stop_reason = 'Stopped after {} navigation candidate attempts'.format(
                attempts)
            break
        if not queued:
            try:
                message, candidates = flow.wait_for_target(require_fresh=True)
                queued, metadata = flow.select_navigation_targets(
                    message, candidates, args.visit_count - len(results),
                    existing=visited + failed_goals)
                if metadata:
                    step_metadata = {
                        'sequence_index': len(results) + 1,
                        'source': 'reperception',
                    }
                    step_metadata.update(metadata)
                    filtering.append(step_metadata)
            except RuntimeError as error:
                recovery_error = str(error)
                recovered = False
                if search_waypoints:
                    for _waypoint_attempt in range(len(search_waypoints)):
                        waypoint = search_waypoints[search_waypoint_index % len(search_waypoints)]
                        search_waypoint_index += 1
                        waypoint_summary = {
                            'sequence_index': len(results) + 1,
                            'waypoint_index': search_waypoint_index,
                            'name': waypoint['name'],
                            'x': waypoint['x'],
                            'y': waypoint['y'],
                            'yaw_deg': math.degrees(waypoint['yaw']),
                        }
                        try:
                            waypoint_pose = send_search_waypoint_goal(args, waypoint)
                            waypoint_summary['status'] = 'SEARCH_WAYPOINT_REACHED'
                            waypoint_summary['pose'] = pose_to_dict(waypoint_pose)
                            message, candidates = flow.wait_for_target_with_timeout(
                                args.recovery_scan_timeout, require_fresh=True)
                            queued, metadata = flow.select_navigation_targets(
                                message, candidates,
                                args.visit_count - len(results),
                                existing=visited + failed_goals)
                            waypoint_summary['result'] = 'TARGETS_REFRESHED'
                            if metadata:
                                step_metadata = {
                                    'sequence_index': len(results) + 1,
                                    'source': 'search_waypoint',
                                    'waypoint': waypoint['name'],
                                }
                                step_metadata.update(metadata)
                                filtering.append(step_metadata)
                            recovered = True
                        except RuntimeError as waypoint_error:
                            waypoint_summary['status'] = 'SEARCH_WAYPOINT_FAILED'
                            waypoint_summary['error'] = str(waypoint_error)
                            recovery_error = str(waypoint_error)
                        search_waypoint_visits.append(waypoint_summary)
                        if recovered:
                            break
                recovery_origin_pose = None
                for scan_index, yaw_offset in enumerate(recovery_scan_offsets, 1):
                    if recovered:
                        break
                    if args.disable_recovery_scan or not visited:
                        break
                    scan_summary = {
                        'sequence_index': len(results) + 1,
                        'scan_index': scan_index,
                        'yaw_offset_deg': math.degrees(yaw_offset),
                    }
                    try:
                        if recovery_origin_pose is None:
                            recovery_origin_pose = flow.current_navigation_pose()
                        scan_pose = send_recovery_scan_goal(
                            args, recovery_origin_pose, yaw_offset)
                        scan_summary['status'] = 'SCAN_GOAL_REACHED'
                        scan_summary['pose'] = pose_to_dict(scan_pose)
                        message, candidates = flow.wait_for_target_with_timeout(
                            args.recovery_scan_timeout, require_fresh=True)
                        queued, metadata = flow.select_navigation_targets(
                            message, candidates,
                            args.visit_count - len(results),
                            existing=visited + failed_goals)
                        scan_summary['result'] = 'TARGETS_REFRESHED'
                        if metadata:
                            step_metadata = {
                                'sequence_index': len(results) + 1,
                                'source': 'recovery_scan',
                            }
                            step_metadata.update(metadata)
                            filtering.append(step_metadata)
                        recovered = True
                    except RuntimeError as scan_error:
                        scan_summary['status'] = 'RECOVERY_SCAN_FAILED'
                        scan_summary['error'] = str(scan_error)
                        recovery_error = str(scan_error)
                    recovery_scans.append(scan_summary)
                    if recovered:
                        break
                if not recovered:
                    stop_reason = recovery_error
                    break

        item = queued.pop(0)
        sequence_index = len(results) + 1
        if (search_waypoints and
                sequence_index not in pre_navigation_repositioned_sequences and
                should_reposition_before_navigation(args, item)):
            pre_navigation_repositioned_sequences.add(sequence_index)
            route_length = navigation_candidate_route_length(item)
            recovered = False
            for waypoint in ordered_waypoints_for_candidate(search_waypoints, item):
                search_waypoint_index += 1
                waypoint_summary = {
                    'sequence_index': sequence_index,
                    'waypoint_index': search_waypoint_index,
                    'source': 'pre_navigation_reposition',
                    'reason': 'routed_path_length_exceeds_threshold',
                    'routed_path_length_m': route_length,
                    'threshold_m': getattr(
                        args, 'search_reposition_path_length', 4.0),
                    'name': waypoint['name'],
                    'x': waypoint['x'],
                    'y': waypoint['y'],
                    'yaw_deg': math.degrees(waypoint['yaw']),
                }
                try:
                    waypoint_pose = send_search_waypoint_goal(args, waypoint)
                    waypoint_summary['status'] = 'SEARCH_WAYPOINT_REACHED'
                    waypoint_summary['pose'] = pose_to_dict(waypoint_pose)
                    message, candidates = flow.wait_for_target_with_timeout(
                        args.recovery_scan_timeout, require_fresh=True)
                    queued, metadata = flow.select_navigation_targets(
                        message, candidates, args.visit_count - len(results),
                        existing=visited + failed_goals)
                    waypoint_summary['result'] = 'TARGETS_REFRESHED'
                    if metadata:
                        step_metadata = {
                            'sequence_index': sequence_index,
                            'source': 'pre_navigation_reposition',
                            'waypoint': waypoint['name'],
                        }
                        step_metadata.update(metadata)
                        filtering.append(step_metadata)
                    recovered = True
                except RuntimeError as waypoint_error:
                    waypoint_summary['status'] = 'SEARCH_WAYPOINT_FAILED'
                    waypoint_summary['error'] = str(waypoint_error)
                search_waypoint_visits.append(waypoint_summary)
                if recovered:
                    break
            if recovered:
                continue

        target_summary = item['summary']
        target_summary['sequence_index'] = sequence_index
        target_summary['mode'] = 'move_base_action'
        attempts += 1
        try:
            confirm_dropoff_before_pickup(args, flow, message, target_summary)
        except RuntimeError as error:
            target_summary['status'] = 'DROPOFF_PRECHECK_FAILED'
            target_summary['error'] = str(error)
            stop_reason = str(error)
            results.append(target_summary)
            break
        try:
            send_move_base_goal(args.move_base_action, item['navigation_pose'],
                                args.navigation_timeout)
        except RuntimeError as error:
            target_summary['status'] = 'NAVIGATION_FAILED'
            target_summary['error'] = str(error)
            failed_goals.append({
                'navigation_pose': item['navigation_pose'],
                'target_pose': item['target_pose'],
            })
            navigation_failures.append(target_summary)
            queued = []
            try:
                queued, metadata = flow.select_navigation_targets(
                    message, candidates, args.visit_count - len(results),
                    existing=visited + failed_goals)
            except RuntimeError:
                continue
            if metadata:
                step_metadata = {
                    'sequence_index': len(results) + 1,
                    'source': 'retry_same_perception',
                }
                step_metadata.update(metadata)
                filtering.append(step_metadata)
            continue
        target_summary['status'] = 'READY_FOR_PICKUP'
        record_pickup_action(args, target_summary, sequence_index)
        if dropoff_enabled_from_args(args):
            try:
                dropoff_items = select_dropoff_navigation_targets_compat(flow, message)
                target_summary['dropoff'] = navigate_to_dropoff(
                    args, dropoff_items, sequence_index)
                target_summary['status'] = 'DROPOFF_REACHED'
            except RuntimeError as error:
                target_summary['dropoff'] = {
                    'sequence_index': sequence_index,
                    'status': 'DROPOFF_FAILED',
                    'error': str(error),
                    'target_classes': dropoff_classes_from_args(args),
                }
                target_summary['status'] = 'DROPOFF_FAILED'
                stop_reason = str(error)
                results.append(target_summary)
                visited.append({
                    'navigation_pose': item['navigation_pose'],
                    'target_pose': item['target_pose'],
                })
                break
        results.append(target_summary)
        visited.append({
            'navigation_pose': item['navigation_pose'],
            'target_pose': item['target_pose'],
        })

        if queued:
            remaining = []
            for item in queued:
                if (flow._is_duplicate_target(item['navigation_pose'],
                                              visited + failed_goals) or
                        flow._is_duplicate_object(item['target_pose'], visited)):
                    continue
                remaining.append(item)
            queued = remaining

    completed = (len(results) == args.visit_count and
                 all(item.get('status') in ('READY_FOR_PICKUP', 'DROPOFF_REACHED')
                     for item in results))
    final_exit = None
    if completed:
        try:
            final_exit = navigate_to_final_exit(args)
        except RuntimeError as error:
            completed = False
            stop_reason = str(error)
            final_exit = {
                'status': 'EXIT_FAILED',
                'error': str(error),
            }
    summary = {
        'status': ('SEQUENTIAL_NAVIGATION_COMPLETE' if completed
                   else 'SEQUENTIAL_NAVIGATION_PARTIAL'),
        'mode': 'sequential_move_base_action',
        'requested_targets': requested_garbage_target_count(args),
        'visited_targets': len(results),
        'completed_dropoffs': completed_dropoff_count(results),
        'garbage_competition_mode': garbage_competition_mode_from_args(args),
        'garbage_quota': garbage_quota_from_args(args),
        'target_class': target_class_label(args),
        'target_classes': target_classes_from_args(args),
        'dropoff_enabled': dropoff_enabled_from_args(args),
        'dropoff_classes': dropoff_classes_from_args(args),
        'selector': args.selector,
        'min_confidence': args.min_confidence,
        'target_dedup_distance_m': args.target_dedup_distance,
        'navigation_attempts': attempts,
        'targets': results,
        'next_state': ('READY_FOR_PICKUP_AFTER_SEQUENTIAL_NAVIGATION' if completed
                       else 'READY_FOR_REPLAN_REMAINING_TARGETS'),
    }
    if filtering:
        summary['candidate_filtering'] = filtering
    if navigation_failures:
        summary['navigation_failures'] = navigation_failures
    if search_waypoint_visits:
        summary['search_waypoints'] = search_waypoint_visits
    if final_exit is not None:
        summary['final_exit'] = final_exit
    if recovery_scans:
        summary['recovery_scans'] = recovery_scans
    if stop_reason:
        summary['stop_reason'] = stop_reason
    summary['competition_audit'] = build_competition_flow_audit(args, summary)
    return summary

def run_room_patrol_navigation(args, flow):
    patrol_waypoints = room_patrol_waypoints(args)
    if not patrol_waypoints:
        raise RuntimeError('Room patrol is not enabled for this scenario')
    results = []
    room_visits = []
    candidate_filtering = []
    navigation_failures = []
    completed_rooms = []
    visited = []
    stop_reason = None
    latest_message = None
    latest_candidates = []

    requested_count = requested_garbage_target_count(args)

    for waypoint_index, waypoint in enumerate(patrol_waypoints, 1):
        if len(results) >= requested_count:
            break
        if waypoint['room'] in completed_rooms:
            continue
        room_summary = {
            'waypoint_index': waypoint_index,
            'sequence_index': len(results) + 1,
            'source': 'room_patrol',
            'name': waypoint['name'],
            'room': waypoint['room'],
            'x': waypoint['x'],
            'y': waypoint['y'],
            'yaw_deg': math.degrees(waypoint['yaw']),
            'target_classes': waypoint.get('classes', []),
        }
        item = None
        try:
            precheck_message, precheck_candidates = flow.wait_for_target_with_timeout(
                args.recovery_scan_timeout, require_fresh=True)
            precheck_room_candidates = room_patrol_candidates(
                flow, precheck_message, waypoint)
            if precheck_room_candidates:
                selected, metadata = flow.select_navigation_targets(
                    precheck_message, precheck_room_candidates, 1, existing=visited)
                if metadata:
                    filter_summary = {
                        'sequence_index': len(results) + 1,
                        'source': 'room_patrol_pre_waypoint_direct',
                        'room': waypoint['room'],
                        'waypoint': waypoint['name'],
                    }
                    filter_summary.update(metadata)
                    candidate_filtering.append(filter_summary)
                item = selected[0]
                latest_message = precheck_message
                latest_candidates = precheck_candidates
                room_summary['status'] = 'ROOM_TARGET_PREDETECTED'
                room_summary['detection_source'] = 'room_pre_waypoint_direct'
                room_summary['pre_waypoint_skip'] = True
            else:
                room_summary['pre_waypoint_detection_error'] = (
                    'fresh detections did not include {}'.format(
                        ','.join(waypoint.get('classes', []))))
        except RuntimeError as precheck_error:
            room_summary['pre_waypoint_detection_error'] = str(precheck_error)

        if item is None:
            try:
                waypoint_pose = send_search_waypoint_goal(args, waypoint)
                room_summary['status'] = 'ROOM_WAYPOINT_REACHED'
                room_summary['pose'] = pose_to_dict(waypoint_pose)
                latest_message, latest_candidates, room_candidates = (
                    wait_for_room_patrol_target(
                        args, flow, waypoint, room_summary, waypoint_pose))
                selected, metadata = flow.select_navigation_targets(
                    latest_message, room_candidates, 1, existing=visited)
                if metadata:
                    filter_summary = {
                        'sequence_index': len(results) + 1,
                        'source': 'room_patrol',
                        'room': waypoint['room'],
                        'waypoint': waypoint['name'],
                    }
                    filter_summary.update(metadata)
                    candidate_filtering.append(filter_summary)
                item = selected[0]
            except RuntimeError as error:
                room_summary['status'] = 'ROOM_PATROL_FAILED'
                room_summary['error'] = str(error)
                room_visits.append(room_summary)
                stop_reason = str(error)
                continue

        target_summary = item['summary']
        sequence_index = len(results) + 1
        target_summary['sequence_index'] = sequence_index
        target_summary['mode'] = 'move_base_action'
        target_summary['source'] = 'room_patrol'
        target_summary['room'] = waypoint['room']
        target_summary['room_waypoint'] = waypoint['name']
        try:
            confirm_dropoff_before_pickup(args, flow, latest_message, target_summary)
        except RuntimeError as error:
            target_summary['status'] = 'DROPOFF_PRECHECK_FAILED'
            target_summary['error'] = str(error)
            results.append(target_summary)
            room_summary['status'] = 'ROOM_DROPOFF_PRECHECK_FAILED'
            room_summary['error'] = str(error)
            room_visits.append(room_summary)
            stop_reason = str(error)
            break
        try:
            send_move_base_goal(args.move_base_action, item['navigation_pose'],
                                args.navigation_timeout)
        except RuntimeError as error:
            target_summary['status'] = 'NAVIGATION_FAILED'
            target_summary['error'] = str(error)
            navigation_failures.append(target_summary)
            room_summary['status'] = 'ROOM_TARGET_NAVIGATION_FAILED'
            room_summary['error'] = str(error)
            room_visits.append(room_summary)
            stop_reason = str(error)
            continue

        target_summary['status'] = 'READY_FOR_PICKUP'
        record_pickup_action(args, target_summary, sequence_index)
        if dropoff_enabled_from_args(args):
            try:
                dropoff_items = select_dropoff_navigation_targets_compat(flow, latest_message)
                target_summary['dropoff'] = navigate_to_dropoff(
                    args, dropoff_items, sequence_index)
                target_summary['status'] = 'DROPOFF_REACHED'
            except RuntimeError as error:
                target_summary['dropoff'] = {
                    'sequence_index': sequence_index,
                    'status': 'DROPOFF_FAILED',
                    'error': str(error),
                    'target_classes': dropoff_classes_from_args(args),
                }
                target_summary['status'] = 'DROPOFF_FAILED'
                results.append(target_summary)
                visited.append({
                    'navigation_pose': item['navigation_pose'],
                    'target_pose': item['target_pose'],
                })
                room_summary['status'] = 'ROOM_DROPOFF_FAILED'
                room_summary['error'] = str(error)
                room_visits.append(room_summary)
                stop_reason = str(error)
                break
        results.append(target_summary)
        visited.append({
            'navigation_pose': item['navigation_pose'],
            'target_pose': item['target_pose'],
        })
        completed_rooms.append(waypoint['room'])
        room_summary['status'] = 'ROOM_COMPLETE'
        room_summary['target_status'] = target_summary['status']
        room_summary['completed_after_dropoff'] = target_summary['status'] == 'DROPOFF_REACHED'
        room_visits.append(room_summary)

    requested_count = requested_garbage_target_count(args)
    completed_count = completed_target_count(args, results)
    skipped_rooms = (skipped_rooms_after_quota(
        args, patrol_waypoints, completed_rooms, room_visits)
                     if completed_count >= requested_count else [])
    required_rooms = min(requested_count, len(unique_patrol_rooms(patrol_waypoints)))
    room_completion_ok = (garbage_competition_mode_from_args(args) or
                          len(completed_rooms) >= required_rooms)
    completed = (completed_count >= requested_count and room_completion_ok and
                 all(item.get('status') in ('READY_FOR_PICKUP', 'DROPOFF_REACHED')
                     for item in results))
    final_exit = None
    if completed:
        try:
            final_exit = navigate_to_final_exit(args)
        except RuntimeError as error:
            completed = False
            stop_reason = str(error)
            final_exit = {'status': 'EXIT_FAILED', 'error': str(error)}

    summary = {
        'status': ('ROOM_PATROL_COMPLETE' if completed else 'ROOM_PATROL_PARTIAL'),
        'mode': 'four_room_waypoint_patrol',
        'requested_targets': requested_count,
        'visited_targets': len(results),
        'completed_dropoffs': completed_dropoff_count(results),
        'garbage_competition_mode': garbage_competition_mode_from_args(args),
        'garbage_quota': garbage_quota_from_args(args),
        'completed_rooms': completed_rooms,
        'target_class': target_class_label(args),
        'target_classes': target_classes_from_args(args),
        'dropoff_enabled': dropoff_enabled_from_args(args),
        'dropoff_classes': dropoff_classes_from_args(args),
        'selector': args.selector,
        'min_confidence': args.min_confidence,
        'target_dedup_distance_m': args.target_dedup_distance,
        'navigation_attempts': len(results) + len(navigation_failures),
        'targets': results,
        'room_waypoints': room_visits,
        'next_state': ('TASK_COMPLETE_EXIT_REACHED' if completed
                       else 'READY_FOR_ROOM_PATROL_REPLAN'),
    }
    if skipped_rooms:
        summary['skipped_rooms_after_quota'] = skipped_rooms
    if candidate_filtering:
        summary['candidate_filtering'] = candidate_filtering
    if navigation_failures:
        summary['navigation_failures'] = navigation_failures
    if final_exit is not None:
        summary['final_exit'] = final_exit
    if stop_reason:
        summary['stop_reason'] = stop_reason
    summary['competition_audit'] = build_competition_flow_audit(args, summary)
    return summary


def initial_room_search_for_targets(args, flow):
    search_waypoints = room_search_waypoints(args)
    search_waypoint_visits = []
    if not (args.send_goal and args.visit_count > 1 and search_waypoints):
        return flow.wait_for_target(), search_waypoint_visits, 'initial_perception'

    try:
        return flow.wait_for_target(), search_waypoint_visits, 'initial_perception'
    except RuntimeError as initial_error:
        last_error = str(initial_error)

    for waypoint_index, waypoint in enumerate(search_waypoints, 1):
        waypoint_summary = {
            'sequence_index': 1,
            'waypoint_index': waypoint_index,
            'source': 'initial_search_waypoint',
            'reason': 'initial_perception_timeout',
            'name': waypoint['name'],
            'x': waypoint['x'],
            'y': waypoint['y'],
            'yaw_deg': math.degrees(waypoint['yaw']),
        }
        try:
            waypoint_pose = send_search_waypoint_goal(args, waypoint)
            waypoint_summary['status'] = 'SEARCH_WAYPOINT_REACHED'
            waypoint_summary['pose'] = pose_to_dict(waypoint_pose)
            result = flow.wait_for_target_with_timeout(
                args.recovery_scan_timeout, require_fresh=True)
            waypoint_summary['result'] = 'TARGETS_REFRESHED'
            search_waypoint_visits.append(waypoint_summary)
            return result, search_waypoint_visits, 'initial_search_waypoint'
        except RuntimeError as waypoint_error:
            waypoint_summary['status'] = 'SEARCH_WAYPOINT_FAILED'
            waypoint_summary['error'] = str(waypoint_error)
            search_waypoint_visits.append(waypoint_summary)
            last_error = str(waypoint_error)

    raise RuntimeError(last_error)


def run_flow(args, flow=None):
    if flow is None:
        flow = TaskFlow(args)
    if room_patrol_enabled(args):
        summary = run_room_patrol_navigation(args, flow)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return
    (message, candidates), initial_search_waypoint_visits, initial_source = (
        initial_room_search_for_targets(args, flow))

    if args.send_goal and args.visit_count > 1:
        summary = run_sequential_navigation(
            args, flow, message, candidates,
            initial_search_waypoint_visits=initial_search_waypoint_visits,
            initial_perception_source=initial_source)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return

    if args.publish_goal or args.send_goal:
        pose, navigation_pose, summary = flow.select_navigation_target(
            message, candidates)
    else:
        target = flow.select_target(candidates)
        pose, summary = flow.compute_goal(message, target)
        navigation_pose = pose

    if args.publish_goal:
        publish_goal(navigation_pose)
        summary['mode'] = 'published_topic'
        summary['status'] = 'NAVIGATION_GOAL_PUBLISHED'
    elif args.send_goal:
        summary['mode'] = 'move_base_action'
        send_move_base_goal(args.move_base_action, navigation_pose,
                            args.navigation_timeout)
        summary['status'] = 'READY_FOR_PICKUP'
        record_pickup_action(args, summary, 1)
        if dropoff_enabled_from_args(args):
            dropoff_items = flow.select_dropoff_navigation_targets(message)
            summary['dropoff'] = navigate_to_dropoff(args, dropoff_items, 1)
            summary['status'] = 'DROPOFF_REACHED'
        final_exit = navigate_to_final_exit(args)
        if final_exit is not None:
            summary['final_exit'] = final_exit
            summary['next_state'] = 'TASK_COMPLETE_EXIT_REACHED'
    summary['competition_audit'] = build_competition_flow_audit(args, summary)
    print(json.dumps(summary, indent=2, sort_keys=True))


def main():
    args = parse_args()
    workspace = '/home/hyc/catkin_wa'
    env = sourced_environment(workspace)
    processes = []
    try:
        if not args.no_launch_stack or args.launch_navigation:
            processes.append(start_roscore(env))
            set_ros_param(env, '/use_sim_time', True)
        if not args.no_launch_stack:
            processes.append(launch_localization_stack(args, env))
        if args.launch_navigation:
            processes.append(launch_navigation_stack(args, env))
        rospy.init_node('robocup_task_flow', anonymous=True)
        flow = TaskFlow(args)
        if processes:
            time.sleep(max(0.0, args.startup_wait))
        run_flow(args, flow)
        return 0
    except (RuntimeError, rospy.ROSException, subprocess.CalledProcessError) as error:
        print('ERROR: {}'.format(error), file=sys.stderr)
        return 1
    finally:
        for process in reversed(processes):
            stop_process(process)


if __name__ == '__main__':
    sys.exit(main())
