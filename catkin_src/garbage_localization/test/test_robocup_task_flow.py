#!/usr/bin/env python3

import math
import os
import sys
import unittest

SCRIPT_DIR = os.path.join(os.path.dirname(__file__), '..', 'scripts')
sys.path.insert(0, os.path.abspath(SCRIPT_DIR))

from run_robocup_task_flow import (  # noqa: E402
    StaticMapSafety,
    TaskFlow,
    dropoff_enabled_from_args,
    final_exit_waypoint,
    send_final_exit_goal_with_retry,
    build_competition_flow_audit,
    initial_room_search_for_targets,
    room_patrol_candidates,
    room_patrol_waypoints,
    room_search_waypoints,
    run_room_patrol_navigation,
    run_sequential_navigation,
    wait_for_room_patrol_target,
    navigate_to_dropoff,
)


class Args(object):
    def __init__(self, **kwargs):
        self.timeout = 5.0
        self.initial_scan_timeout = 0.25
        self.disable_initial_scan = False
        self.send_goal = True
        self.visit_count = 1
        self.target_class = 'bottle'
        self.target_classes = None
        self.dropoff_classes = 'trash_bin'
        self.dropoff_approach_distance = 0.75
        self.pickup_action = 'dry-run'
        self.dropoff_action = 'dry-run'
        self.enable_dropoff = False
        self.disable_dropoff = False
        self.final_exit = 'none'
        self.final_exit_timeout = 1.0
        self.object_topic = '/garbage_localization/objects'
        self.min_confidence = 0.0
        self.allow_behind = False
        self.scenario = 'three'
        self.navigation_frame = 'map'
        self.move_base_action = '/move_base'
        self.navigation_timeout = 1.0
        self.search_waypoints = 'none'
        self.search_waypoint_timeout = 1.0
        self.room_patrol = 'off'
        self.recovery_scan_timeout = 1.0
        self.disable_recovery_scan = False
        self.selector = 'nearest'
        self.target_dedup_distance = 0.45
        self.garbage_competition_mode = False
        self.garbage_quota = 3
        self.navigation_min_obstacle_distance = 0.55
        self.__dict__.update(kwargs)


class ParseArgsCompetitionModeTest(unittest.TestCase):
    def test_garbage_competition_mode_forces_three_garbage_rule_settings(self):
        import run_robocup_task_flow
        original_argv = sys.argv[:]
        try:
            sys.argv = [
                'run_robocup_task_flow.py',
                '--scenario', 'four_paper_ball',
                '--send-goal',
                '--garbage-competition-mode',
                '--visit-count', '4',
                '--target-classes', 'trash_bin',
            ]
            args = run_robocup_task_flow.parse_args()
        finally:
            sys.argv = original_argv

        self.assertTrue(args.garbage_competition_mode)
        self.assertEqual(3, args.garbage_quota)
        self.assertEqual(3, args.visit_count)
        self.assertEqual(['bottle', 'paper_ball', 'box'], args.target_classes)
        self.assertTrue(dropoff_enabled_from_args(args))

    def test_garbage_competition_mode_rejects_disabled_dropoff(self):
        import run_robocup_task_flow
        original_argv = sys.argv[:]
        try:
            sys.argv = [
                'run_robocup_task_flow.py',
                '--scenario', 'four_paper_ball',
                '--send-goal',
                '--garbage-competition-mode',
                '--disable-dropoff',
            ]
            with self.assertRaises(SystemExit):
                run_robocup_task_flow.parse_args()
        finally:
            sys.argv = original_argv


class TaskFlowInitialScanTest(unittest.TestCase):
    def make_flow(self, args):
        flow = TaskFlow.__new__(TaskFlow)
        flow.args = args
        flow.min_targets = 1
        return flow

    def test_initial_scan_refreshes_detection_after_first_timeout(self):
        flow = self.make_flow(Args())
        wait_calls = []
        scan_offsets = []

        def wait_once(timeout, require_fresh=False):
            wait_calls.append((timeout, require_fresh))
            if len(wait_calls) == 1:
                raise RuntimeError('initial timeout')
            return 'message', ['candidate']

        flow._wait_for_target_once = wait_once
        flow.send_initial_scan_goal = scan_offsets.append

        message, candidates = flow.wait_for_target()

        self.assertEqual('message', message)
        self.assertEqual(['candidate'], candidates)
        self.assertEqual([(5.0, False), (0.25, True)], wait_calls)
        self.assertEqual(1, len(scan_offsets))
        self.assertAlmostEqual(math.radians(45.0), scan_offsets[0])

    def test_initial_scan_is_not_used_for_fresh_reperception(self):
        flow = self.make_flow(Args())
        flow._wait_for_target_once = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError('fresh timeout'))
        flow.send_initial_scan_goal = lambda _yaw: self.fail(
            'fresh re-perception must use recovery scan path, not initial scan')

        with self.assertRaises(RuntimeError):
            flow.wait_for_target(require_fresh=True)

    def test_initial_scan_respects_disable_flag(self):
        flow = self.make_flow(Args(disable_initial_scan=True))
        flow._wait_for_target_once = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError('disabled timeout'))
        flow.send_initial_scan_goal = lambda _yaw: self.fail(
            'disabled initial scan should not send scan goals')

        with self.assertRaises(RuntimeError):
            flow.wait_for_target()
    def test_candidate_objects_respects_min_confidence(self):
        flow = self.make_flow(Args(min_confidence=0.05))

        def obj(class_name, confidence, x):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = confidence
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        message = type('MessageStub', (), {})()
        message.objects = [
            obj('bottle', 0.049, 1.0),
            obj('bottle', 0.05, 2.0),
            obj('paper', 0.99, 3.0),
        ]

        candidates = flow._candidate_objects(message)

        self.assertEqual(1, len(candidates))
        self.assertEqual(0.05, candidates[0]['object'].confidence)

    def test_candidate_objects_accepts_configured_paper_ball_target(self):
        flow = self.make_flow(Args(target_class='paper_ball', min_confidence=0.10))

        def obj(class_name, confidence, x):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = confidence
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        message = type('MessageStub', (), {})()
        message.objects = [
            obj('bottle', 0.99, 1.0),
            obj('paper_ball', 0.09, 2.0),
            obj('paper_ball', 0.10, 3.0),
            obj('box', 0.99, 4.0),
        ]

        candidates = flow._candidate_objects(message)

        self.assertEqual(1, len(candidates))
        self.assertEqual('paper_ball', candidates[0]['object'].class_name)
        self.assertEqual(0.10, candidates[0]['object'].confidence)
        self.assertEqual(3.0, candidates[0]['object'].position.x)

    def test_candidate_objects_keeps_trash_bin_out_of_garbage_targets(self):
        flow = self.make_flow(Args(target_classes=['bottle', 'paper_ball', 'box']))

        def obj(class_name, confidence, x):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = confidence
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        message = type('MessageStub', (), {})()
        message.objects = [
            obj('bottle', 0.99, 1.0),
            obj('paper_ball', 0.99, 2.0),
            obj('box', 0.99, 3.0),
            obj('trash_bin', 0.99, 4.0),
        ]

        candidates = flow._candidate_objects(message)

        self.assertEqual(['bottle', 'paper_ball', 'box'],
                         [c['object'].class_name for c in candidates])

    def test_dropoff_objects_selects_trash_bin_separately(self):
        flow = self.make_flow(Args(target_classes=['bottle', 'paper_ball', 'box'],
                                   dropoff_classes=['trash_bin']))

        def obj(class_name, confidence, x):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = confidence
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        message = type('MessageStub', (), {})()
        message.objects = [obj('box', 0.99, 1.0), obj('trash_bin', 0.99, 2.0)]

        dropoff = flow.select_dropoff_target(message)

        self.assertEqual('trash_bin', dropoff['object'].class_name)

    def test_four_paper_ball_dropoff_rejects_false_cached_bin(self):
        flow = self.make_flow(Args(scenario='four_paper_ball',
                                   navigation_frame='map'))

        def obj(class_name, confidence, x, y):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = confidence
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = y
            item.position.z = 0.0
            item.bbox_x = 0.0
            item.bbox_y = 0.0
            item.bbox_width = 1.0
            item.bbox_height = 1.0
            return item

        def message(x, y):
            item = type('MessageStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.objects = [obj('trash_bin', 0.99, x, y)]
            return item

        stale = obj('trash_bin', 0.99, 8.071, 7.123)
        flow._latest_dropoff = {
            'object': stale,
            'range': math.hypot(stale.position.x, stale.position.y),
            'cached_in_navigation_frame': True,
        }

        with self.assertRaises(RuntimeError):
            flow.select_dropoff_target(message(8.071, 7.123))
        self.assertIsNone(flow._latest_dropoff)

        dropoff = flow.select_dropoff_target(message(4.157, 4.770))

        self.assertEqual('trash_bin', dropoff['object'].class_name)
        self.assertAlmostEqual(4.157, dropoff['object'].position.x)
        self.assertAlmostEqual(4.770, dropoff['object'].position.y)

    def test_dropoff_enabled_for_multiclass_but_not_legacy_bottle_default(self):
        self.assertFalse(dropoff_enabled_from_args(Args()))
        self.assertFalse(dropoff_enabled_from_args(Args(target_classes=['bottle'])))
        self.assertTrue(dropoff_enabled_from_args(Args(
            target_classes=['bottle', 'paper_ball', 'box'])))
        self.assertTrue(dropoff_enabled_from_args(Args(scenario='four_paper_ball')))
        self.assertTrue(dropoff_enabled_from_args(Args(enable_dropoff=True)))
        self.assertFalse(dropoff_enabled_from_args(Args(
            scenario='four_paper_ball', disable_dropoff=True)))

    def test_room_search_waypoints_auto_only_for_four_occlusion(self):
        waypoints = room_search_waypoints(Args(scenario='four_occlusion',
                                               search_waypoints='auto'))

        self.assertEqual(4, len(waypoints))
        self.assertEqual(['living_room_center', 'bedroom_center',
                          'dining_room_center', 'kitchen_center'],
                         [item['name'] for item in waypoints])
        self.assertEqual([], room_search_waypoints(Args(scenario='three',
                                                        search_waypoints='auto')))
        self.assertEqual([], room_search_waypoints(Args(scenario='four_occlusion',
                                                        search_waypoints='none')))

    def test_room_search_waypoints_accepts_custom_spec(self):
        waypoints = room_search_waypoints(Args(
            scenario='four_occlusion',
            search_waypoints='living:6.75:2.2:90,kitchen:6.8:6.25:-90'))

        self.assertEqual(['living', 'kitchen'], [item['name'] for item in waypoints])
        self.assertAlmostEqual(6.75, waypoints[0]['x'])
        self.assertAlmostEqual(math.radians(-90.0), waypoints[1]['yaw'])

    def test_initial_perception_timeout_uses_room_search_waypoint(self):
        args = Args(visit_count=4,
                    scenario='four_occlusion',
                    search_waypoints='living:6.75:2.2:90,kitchen:6.8:6.25:-90',
                    recovery_scan_timeout=0.25,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        wait_calls = []
        sent_waypoints = []

        def wait_for_target(require_fresh=False):
            self.assertFalse(require_fresh)
            raise RuntimeError('initial timeout')

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            wait_calls.append((timeout, require_fresh))
            self.assertEqual(args.recovery_scan_timeout, timeout)
            self.assertTrue(require_fresh)
            return 'message', ['candidate']

        flow.wait_for_target = wait_for_target
        flow.wait_for_target_with_timeout = wait_for_target_with_timeout

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        try:
            def fake_send_move_base_goal(_action_name, sent_pose, _timeout):
                sent_waypoints.append((sent_pose.pose.position.x,
                                       sent_pose.pose.position.y))

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            result, visits, source = initial_room_search_for_targets(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual(('message', ['candidate']), result)
        self.assertEqual('initial_search_waypoint', source)
        self.assertEqual([(args.recovery_scan_timeout, True)], wait_calls)
        self.assertEqual([(6.75, 2.2)], sent_waypoints)
        self.assertEqual(1, len(visits))
        self.assertEqual('initial_search_waypoint', visits[0]['source'])
        self.assertEqual('TARGETS_REFRESHED', visits[0]['result'])

    def test_search_waypoint_refreshes_targets_before_recovery_scan(self):
        args = Args(visit_count=2,
                    scenario='four_occlusion',
                    search_waypoints='living:6.75:2.2:90',
                    disable_recovery_scan=False,
                    recovery_scan_timeout=0.25,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55,
                    move_base_action='/move_base')
        flow = self.make_flow(args)
        wait_calls = []
        sent_scan_goals = []
        sent_waypoints = []

        def pose(name):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = float(len(sent_waypoints) + len(wait_calls))
            item.pose.position.y = 0.0
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            item.name = name
            return item

        def selected_entry(name):
            nav_pose = pose(name)
            return {
                'navigation_pose': nav_pose,
                'target_pose': nav_pose,
                'summary': {'target': {'name': name}},
            }

        selection_count = [0]

        def select_navigation_targets(_message, _candidates, _count, existing=None):
            selection_count[0] += 1
            return [selected_entry('target-{}'.format(selection_count[0]))], {}

        flow.select_navigation_targets = select_navigation_targets
        flow.wait_for_target = lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError('fresh timeout'))

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            wait_calls.append((timeout, require_fresh))
            self.assertEqual(args.recovery_scan_timeout, timeout)
            self.assertTrue(require_fresh)
            return 'message', ['candidate']

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._is_duplicate_target = lambda *_args, **_kwargs: False
        flow._is_duplicate_object = lambda *_args, **_kwargs: False
        flow.current_navigation_pose = lambda: pose('scan-current')

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        original_send_recovery_scan_goal = run_robocup_task_flow.send_recovery_scan_goal
        try:
            def fake_send_move_base_goal(_action_name, sent_pose, _timeout):
                sent_waypoints.append(sent_pose)

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            run_robocup_task_flow.send_recovery_scan_goal = (
                lambda *_args, **_kwargs: sent_scan_goals.append(1))

            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal
            run_robocup_task_flow.send_recovery_scan_goal = original_send_recovery_scan_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual(2, summary['visited_targets'])
        self.assertEqual(1, len(summary['search_waypoints']))
        self.assertEqual('TARGETS_REFRESHED', summary['search_waypoints'][0]['result'])
        self.assertEqual([], sent_scan_goals)

        args = Args(visit_count=3,
                    disable_recovery_scan=False,
                    recovery_scan_timeout=0.25,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        selection_calls = []
        wait_calls = []
        scan_offsets = []

        def pose(name):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = float(len(selection_calls) + len(wait_calls))
            item.pose.position.y = 0.0
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            item.name = name
            return item

        def selected_entry(name):
            nav_pose = pose(name)
            return {
                'navigation_pose': nav_pose,
                'target_pose': nav_pose,
                'summary': {'target': {'name': name}},
            }

        def select_navigation_targets(_message, _candidates, _count, existing=None):
            selection_calls.append(existing or [])
            return [selected_entry('target-{}'.format(len(selection_calls)))], {}

        def wait_for_target(require_fresh=False):
            self.assertTrue(require_fresh)
            raise RuntimeError('fresh timeout')

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            wait_calls.append((timeout, require_fresh))
            self.assertEqual(args.recovery_scan_timeout, timeout)
            self.assertTrue(require_fresh)
            if len(wait_calls) in (1, 2):
                return 'message', ['candidate']
            raise RuntimeError('scan timeout')

        flow.select_navigation_targets = select_navigation_targets
        flow.wait_for_target = wait_for_target
        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._is_duplicate_target = lambda *_args, **_kwargs: False
        flow._is_duplicate_object = lambda *_args, **_kwargs: False
        current_scan_poses = []

        def current_navigation_pose():
            current_pose = pose('scan-current-{}'.format(
                len(current_scan_poses) + 1))
            current_scan_poses.append(current_pose)
            return current_pose

        flow.current_navigation_pose = current_navigation_pose

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        original_send_recovery_scan_goal = run_robocup_task_flow.send_recovery_scan_goal
        sent_scan_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = lambda *_args, **_kwargs: None

            def fake_send_recovery_scan_goal(_args, nav_pose, yaw_offset):
                sent_scan_poses.append(nav_pose.name)
                scan_offsets.append(yaw_offset)
                return nav_pose

            run_robocup_task_flow.send_recovery_scan_goal = fake_send_recovery_scan_goal

            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal
            run_robocup_task_flow.send_recovery_scan_goal = original_send_recovery_scan_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual(3, summary['visited_targets'])
        self.assertEqual([math.radians(45.0), math.radians(45.0)], scan_offsets)
        self.assertEqual(['scan-current-1', 'scan-current-2'], sent_scan_poses)
        self.assertEqual([1, 1], [item['scan_index'] for item in summary['recovery_scans']])

    def test_recovery_scan_attempts_share_one_origin_pose(self):
        args = Args(visit_count=2,
                    disable_recovery_scan=False,
                    recovery_scan_timeout=0.25,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        wait_calls = []
        current_scan_poses = []
        sent_scan_poses = []

        def pose(name):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = 0.0
            item.pose.position.y = 0.0
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            item.name = name
            return item

        def selected_entry(name):
            nav_pose = pose(name)
            return {
                'navigation_pose': nav_pose,
                'target_pose': nav_pose,
                'summary': {'target': {'name': name}, 'approach': {}},
            }

        flow.select_navigation_targets = (
            lambda *_args, **_kwargs: ([selected_entry('target')], {}))
        flow.wait_for_target = lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError('fresh timeout'))

        def wait_for_target_with_timeout(*_args, **_kwargs):
            wait_calls.append(1)
            if len(wait_calls) == 1:
                raise RuntimeError('scan timeout 1')
            return 'message', ['candidate']

        def current_navigation_pose():
            current_pose = pose('scan-current-{}'.format(
                len(current_scan_poses) + 1))
            current_scan_poses.append(current_pose)
            return current_pose

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow.current_navigation_pose = current_navigation_pose

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        original_send_recovery_scan_goal = run_robocup_task_flow.send_recovery_scan_goal
        try:
            run_robocup_task_flow.send_move_base_goal = lambda *_args, **_kwargs: None

            def fake_send_recovery_scan_goal(_args, nav_pose, _yaw_offset):
                sent_scan_poses.append(nav_pose.name)
                return nav_pose

            run_robocup_task_flow.send_recovery_scan_goal = fake_send_recovery_scan_goal

            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal
            run_robocup_task_flow.send_recovery_scan_goal = original_send_recovery_scan_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual(['scan-current-1', 'scan-current-1'], sent_scan_poses)
        self.assertEqual(1, len(current_scan_poses))

    def test_long_route_repositions_to_nearest_waypoint_before_navigation(self):
        args = Args(visit_count=1,
                    scenario='four_occlusion',
                    search_waypoints='far:10:0:0,near:1:0:0',
                    search_reposition_path_length=4.0,
                    disable_recovery_scan=True,
                    recovery_scan_timeout=0.25,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        wait_calls = []
        sent_waypoints = []

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        def entry(name, route_length):
            nav_pose = pose(2.0, 0.0)
            target_pose = pose(1.2, 0.0)
            return {
                'navigation_pose': nav_pose,
                'target_pose': target_pose,
                'summary': {
                    'target': {
                        'name': name,
                        'navigation_position': {
                            'frame_id': 'map', 'x': 1.2, 'y': 0.0, 'z': 0.0,
                        },
                    },
                    'approach': {'routed_path_length_m': route_length},
                },
            }

        selections = [[entry('long-route', 5.0)], [entry('refreshed', 1.0)]]

        def select_navigation_targets(_message, _candidates, _count, existing=None):
            return selections.pop(0), {}

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            wait_calls.append((timeout, require_fresh))
            self.assertEqual(args.recovery_scan_timeout, timeout)
            self.assertTrue(require_fresh)
            return 'fresh-message', ['fresh-candidate']

        flow.select_navigation_targets = select_navigation_targets
        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._is_duplicate_target = lambda *_args, **_kwargs: False
        flow._is_duplicate_object = lambda *_args, **_kwargs: False

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        try:
            def fake_send_move_base_goal(_action_name, sent_pose, _timeout):
                sent_waypoints.append((sent_pose.pose.position.x,
                                       sent_pose.pose.position.y))

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual('refreshed', summary['targets'][0]['target']['name'])
        self.assertEqual([(1.0, 0.0), (2.0, 0.0)], sent_waypoints)
        self.assertEqual(1, len(wait_calls))
        self.assertEqual('pre_navigation_reposition',
                         summary['search_waypoints'][0]['source'])
        self.assertEqual('near', summary['search_waypoints'][0]['name'])

    def test_sequential_navigation_sends_dropoff_after_pickup_when_enabled(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    disable_recovery_scan=True,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        garbage_pose = pose(1.0, 0.0)
        dropoff_pose = pose(2.0, 0.0)
        flow.select_navigation_targets = lambda *_args, **_kwargs: ([{
            'navigation_pose': garbage_pose,
            'target_pose': pose(1.2, 0.0),
            'summary': {'target': {'name': 'bottle'}, 'approach': {}},
        }], {})
        dropoff_calls = []

        def select_dropoff_navigation_target(_message):
            dropoff_calls.append(True)
            return {
                'navigation_pose': dropoff_pose,
                'target_pose': pose(2.2, 0.0),
                'summary': {'target': {'name': 'trash_bin'}, 'approach': {}},
            }

        flow.select_dropoff_navigation_target = select_dropoff_navigation_target
        precheck_calls = []
        flow.confirm_dropoff_before_pickup = lambda _message: (
            precheck_calls.append(True) or
            {'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'})

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual([garbage_pose, dropoff_pose], sent_poses)
        self.assertEqual([True], dropoff_calls)
        self.assertTrue(summary['dropoff_enabled'])
        self.assertEqual(['trash_bin'], summary['dropoff_classes'])
        self.assertEqual('DROPOFF_REACHED', summary['targets'][0]['status'])
        self.assertEqual('DROPOFF_REACHED', summary['targets'][0]['dropoff']['status'])
        self.assertEqual('COMPETITION_AUDIT_PASSED',
                         summary['competition_audit']['status'])
        self.assertEqual('PICKUP_DRY_RUN_COMPLETE',
                         summary['targets'][0]['pickup_action']['status'])
        self.assertFalse(summary['targets'][0]['pickup_action']['hardware_commanded'])
        self.assertEqual('DROPOFF_DRY_RUN_COMPLETE',
                         summary['targets'][0]['dropoff']['dropoff_action']['status'])
        self.assertFalse(
            summary['targets'][0]['dropoff']['dropoff_action']['hardware_commanded'])

    def test_action_hooks_can_be_disabled_without_changing_navigation(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    pickup_action='none',
                    dropoff_action='none',
                    disable_recovery_scan=True,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        garbage_pose = pose(1.0, 0.0)
        dropoff_pose = pose(2.0, 0.0)
        flow.select_navigation_targets = lambda *_args, **_kwargs: ([{
            'navigation_pose': garbage_pose,
            'target_pose': pose(1.2, 0.0),
            'summary': {'target': {'name': 'bottle'}, 'approach': {}},
        }], {})
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': dropoff_pose,
            'target_pose': pose(2.2, 0.0),
            'summary': {'target': {'name': 'trash_bin'}, 'approach': {}},
        }
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual([garbage_pose, dropoff_pose], sent_poses)
        self.assertNotIn('pickup_action', summary['targets'][0])
        self.assertNotIn('dropoff_action', summary['targets'][0]['dropoff'])

    def test_navigate_to_dropoff_records_dry_run_action(self):
        args = Args(dropoff_action='dry-run')
        dropoff_pose = type('PoseStampedStub', (), {})()
        dropoff_item = {
            'navigation_pose': dropoff_pose,
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        try:
            run_robocup_task_flow.send_move_base_goal = lambda *_args, **_kwargs: None
            result = navigate_to_dropoff(args, dropoff_item, 3)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('DROPOFF_REACHED', result['status'])
        self.assertEqual('DROPOFF_DRY_RUN_COMPLETE',
                         result['dropoff_action']['status'])
        self.assertEqual(3, result['dropoff_action']['sequence_index'])
        self.assertEqual('trash_bin', result['dropoff_action']['target_class'])
        self.assertFalse(result['dropoff_action']['hardware_commanded'])

    def test_four_paper_ball_auto_exit_uses_non_start_opening(self):
        args = Args(scenario='four_paper_ball', final_exit='auto')

        waypoint = final_exit_waypoint(args)

        self.assertEqual('wp6_left_dining_room_exit', waypoint['name'])
        self.assertAlmostEqual(0.190553, waypoint['x'])
        self.assertAlmostEqual(7.89226, waypoint['y'])
        self.assertAlmostEqual(-3.084572395452059, waypoint['yaw'])
        self.assertEqual('bottom_living_room_entrance',
                         waypoint['excluded_start_opening'])

    def test_final_exit_retries_same_wp6_goal_after_costmap_clears(self):
        import run_robocup_task_flow
        args = Args(final_exit_attempts=4, move_base_action='/move_base')
        pose = type('PoseStampedStub', (), {})()
        calls = []
        clears = []

        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        original_clear_move_base_costmaps = run_robocup_task_flow.clear_move_base_costmaps
        original_sleep = run_robocup_task_flow.time.sleep
        try:
            def send_goal(action_name, sent_pose, timeout):
                calls.append((action_name, sent_pose, timeout))
                if len(calls) < 4:
                    raise RuntimeError('move_base finished with state 4')

            run_robocup_task_flow.send_move_base_goal = send_goal
            run_robocup_task_flow.clear_move_base_costmaps = lambda: clears.append(True)
            run_robocup_task_flow.time.sleep = lambda _seconds: None

            attempts = send_final_exit_goal_with_retry(args, pose, 90.0)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal
            run_robocup_task_flow.clear_move_base_costmaps = original_clear_move_base_costmaps
            run_robocup_task_flow.time.sleep = original_sleep

        self.assertEqual(4, len(calls))
        self.assertEqual(3, len(clears))
        self.assertTrue(all(call[1] is pose for call in calls))
        self.assertEqual('EXIT_REACHED', attempts[-1]['status'])
        self.assertEqual('clear_costmaps', attempts[-1]['recovery'])

    def test_sequential_navigation_sends_final_exit_after_completed_dropoffs(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    disable_recovery_scan=True,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        garbage_pose = pose(1.0, 0.0)
        dropoff_pose = pose(2.0, 0.0)
        flow.select_navigation_targets = lambda *_args, **_kwargs: ([{
            'navigation_pose': garbage_pose,
            'target_pose': pose(1.2, 0.0),
            'summary': {'target': {'name': 'bottle'}, 'approach': {}},
        }], {})
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': dropoff_pose,
            'target_pose': pose(2.2, 0.0),
            'summary': {'target': {'name': 'trash_bin'}, 'approach': {}},
        }
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual([garbage_pose, dropoff_pose], sent_poses[:2])
        self.assertEqual(3, len(sent_poses))
        self.assertAlmostEqual(0.190553, sent_poses[2].pose.position.x)
        self.assertAlmostEqual(7.89226, sent_poses[2].pose.position.y)
        self.assertEqual('EXIT_REACHED', summary['final_exit']['status'])
        self.assertEqual('wp6_left_dining_room_exit', summary['final_exit']['name'])
        self.assertEqual('COMPETITION_AUDIT_PASSED',
                         summary['competition_audit']['status'])

    def test_sequential_navigation_does_not_exit_when_partial(self):
        args = Args(visit_count=2,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    disable_dropoff=True,
                    disable_recovery_scan=True,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        flow.select_navigation_targets = lambda *_args, **_kwargs: ([{
            'navigation_pose': pose(1.0, 0.0),
            'target_pose': pose(1.2, 0.0),
            'summary': {'target': {'name': 'only-one'}, 'approach': {}},
        }], {})
        flow.wait_for_target = lambda **_kwargs: (_ for _ in ()).throw(
            RuntimeError('no second target'))

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_PARTIAL', summary['status'])
        self.assertEqual(1, len(sent_poses))
        self.assertNotIn('final_exit', summary)

    def test_failed_navigation_blacklists_object_for_retry(self):
        args = Args(visit_count=1,
                    disable_recovery_scan=True,
                    recovery_scan_timeout=0.25,
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        selection_existing_counts = []

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        failed_target_pose = pose(1.0, 0.0)
        successful_target_pose = pose(2.0, 0.0)

        def entry(name, target_pose, navigation_x):
            return {
                'navigation_pose': pose(navigation_x, 1.0),
                'target_pose': target_pose,
                'summary': {'target': {'name': name}},
            }

        selections = [
            [entry('first-fails', failed_target_pose, 1.0)],
            [entry('same-object-would-repeat', failed_target_pose, 1.5)],
            [entry('second-succeeds', successful_target_pose, 2.0)],
        ]

        def select_navigation_targets(_message, _candidates, _count, existing=None):
            existing = existing or []
            selection_existing_counts.append(len(existing))
            while selections:
                candidate = selections.pop(0)[0]
                if flow._is_duplicate_object(candidate['target_pose'], existing):
                    continue
                return [candidate], {}
            raise RuntimeError('no selectable target')

        def wait_for_target(require_fresh=False):
            self.assertTrue(require_fresh)
            return 'message', ['candidate']

        flow.select_navigation_targets = select_navigation_targets
        flow.wait_for_target = wait_for_target
        flow._is_duplicate_target = lambda *_args, **_kwargs: False

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        calls = []
        try:
            def fake_send_move_base_goal(_action_name, _pose, _timeout):
                calls.append(_pose)
                if len(calls) == 1:
                    raise RuntimeError('move_base finished with state 4')

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            summary = run_sequential_navigation(args, flow, 'message', ['candidate'])
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('SEQUENTIAL_NAVIGATION_COMPLETE', summary['status'])
        self.assertEqual(2, len(calls))
        self.assertEqual('second-succeeds', summary['targets'][0]['target']['name'])
        self.assertGreaterEqual(selection_existing_counts[-1], 1)

    def make_grid_safety(self, width, height, blocked_cells):
        safety = StaticMapSafety.__new__(StaticMapSafety)
        safety.resolution = 1.0
        safety.origin = [0.0, 0.0, 0.0]
        safety.width = width
        safety.height = height
        safety.minimum_obstacle_distance = 0.1
        safety.path_minimum_obstacle_distance = 0.1
        safety.forbidden_path_regions = []
        safety._blocked_cache = bytearray(width * height)
        safety.pixels = bytearray([254] * width * height)
        for px, py in blocked_cells:
            safety._blocked_cache[py * width + px] = 1
            safety.pixels[py * width + px] = 0
        return safety

    def test_cached_dropoff_goal_uses_current_robot_pose(self):
        args = Args(approach_distance=1.05,
                    dropoff_approach_distance=1.10,
                    navigation_frame='map')
        flow = self.make_flow(args)
        flow._map_safety = None

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        obj = type('ObjectStub', (), {})()
        obj.class_name = 'trash_bin'
        obj.confidence = 0.9
        obj.position = type('PositionStub', (), {})()
        obj.position.x = 5.0
        obj.position.y = 5.0
        obj.position.z = 0.4
        obj.bbox_x = 1.0
        obj.bbox_y = 2.0
        obj.bbox_width = 3.0
        obj.bbox_height = 4.0
        entry = {
            'object': obj,
            'range': math.hypot(obj.position.x, obj.position.y),
            'cached_in_navigation_frame': True,
        }
        start_pose = pose(5.0, 2.0)
        flow.current_navigation_pose = lambda: self.fail(
            'start_pose should be used when provided')

        navigation_entry = flow._navigation_entry(
            type('MessageStub', (), {'header': type('HeaderStub', (), {'frame_id': 'base_link'})()})(),
            entry,
            angle_offset=0.0,
            legacy=True,
            approach_distance=args.dropoff_approach_distance,
            start_pose=start_pose)
        _pose, navigation_pose, target_pose, summary, _is_safe, _distance, _path_distance, _path_length = navigation_entry

        self.assertEqual('map', navigation_pose.header.frame_id)
        self.assertAlmostEqual(5.0, navigation_pose.pose.position.x)
        self.assertAlmostEqual(3.9, navigation_pose.pose.position.y)
        self.assertAlmostEqual(5.0, target_pose.pose.position.x)
        self.assertAlmostEqual(5.0, target_pose.pose.position.y)
        self.assertEqual('cached_navigation_frame_dropoff', summary['target']['source'])

    def test_static_map_path_safety_allows_routed_corridor(self):
        safety = self.make_grid_safety(5, 5, {(2, 2)})

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            return item

        is_safe, distance = safety.is_path_safe(pose(0.5, 2.5), pose(4.5, 2.5))

        self.assertTrue(is_safe)
        self.assertGreaterEqual(distance, safety.minimum_obstacle_distance)

    def test_static_map_path_safety_rejects_disconnected_goal(self):
        safety = self.make_grid_safety(5, 5, {(2, y) for y in range(5)})

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            return item

        is_safe, distance = safety.is_path_safe(pose(0.5, 2.5), pose(4.5, 2.5))

        self.assertFalse(is_safe)
        self.assertLess(distance, safety.minimum_obstacle_distance)

    def test_static_map_path_safety_rejects_forbidden_outside_route(self):
        safety = self.make_grid_safety(6, 3, set())
        safety.forbidden_path_regions = [{
            'name': 'outside_bottom_living_room_entrance',
            'min_x': 2.0,
            'max_x': 3.0,
            'min_y': 1.0,
            'max_y': 2.0,
        }]

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            return item

        is_safe, distance, path_length, metadata = safety.path_metrics(
            pose(0.5, 1.5), pose(5.5, 1.5))

        self.assertFalse(is_safe)
        self.assertGreater(path_length, 0.0)
        self.assertGreaterEqual(distance, safety.minimum_obstacle_distance)
        self.assertEqual('forbidden_path_region', metadata['reason'])
        self.assertEqual('outside_bottom_living_room_entrance', metadata['region'])

    def test_static_map_path_safety_allows_indoor_route_around_forbidden_region(self):
        safety = self.make_grid_safety(6, 3, set())
        safety.forbidden_path_regions = [{
            'name': 'outside_bottom_living_room_entrance',
            'min_x': 2.0,
            'max_x': 3.0,
            'min_y': 1.0,
            'max_y': 2.0,
        }]

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            return item

        is_safe, distance, _path_length, metadata = safety.path_metrics(
            pose(0.5, 2.5), pose(5.5, 2.5))

        self.assertTrue(is_safe)
        self.assertGreaterEqual(distance, safety.minimum_obstacle_distance)
        self.assertIsNone(metadata)

    def test_select_navigation_targets_records_forbidden_route_rejection(self):
        args = Args(selector='nearest',
                    target_dedup_distance=0.25,
                    approach_distance=0.65,
                    navigation_min_obstacle_distance=0.55,
                    navigation_path_min_obstacle_distance=0.30,
                    disable_navigation_path_filter=False)
        flow = self.make_flow(args)
        flow._map_safety = object()

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        def obj(name, x):
            item = type('ObjectStub', (), {})()
            item.name = name
            item.class_name = 'bottle'
            item.confidence = 0.9
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        candidates = [
            {'object': obj('outside-route', 1.0), 'range': 1.0},
            {'object': obj('inside-route', 2.0), 'range': 2.0},
        ]
        message = type('MessageStub', (), {})()
        flow.current_navigation_pose = lambda: pose(0.0, 0.0)
        flow._is_duplicate_object = lambda *_args, **_kwargs: False
        flow._is_duplicate_target = lambda *_args, **_kwargs: False

        def navigation_entries(_message, entry, start_pose=None):
            self.assertIsNotNone(start_pose)
            nav_pose = pose(entry['range'], 0.0)
            summary = {
                'target': {'name': entry['object'].name},
                'approach': {'distance_m': 0.65},
            }
            if entry['object'].name == 'outside-route':
                summary['approach']['path_filter'] = {
                    'reason': 'forbidden_path_region',
                    'region': 'outside_bottom_living_room_entrance',
                }
                yield (nav_pose, nav_pose, nav_pose, summary, False, 0.80, 0.80, 3.0)
            else:
                yield (nav_pose, nav_pose, nav_pose, summary, True, 0.80, 0.80, 2.0)

        flow._navigation_entries = navigation_entries

        selected, metadata = flow.select_navigation_targets(
            message, candidates, 1)

        self.assertEqual('inside-route', selected[0]['summary']['target']['name'])
        rejected = metadata['rejected_navigation_candidates'][0]
        self.assertEqual('forbidden_path_region', rejected['path_filter']['reason'])
        self.assertEqual('outside_bottom_living_room_entrance',
                         rejected['path_filter']['region'])

    def test_select_navigation_targets_rejects_path_unsafe_candidate(self):
        args = Args(selector='nearest',
                    target_dedup_distance=0.25,
                    approach_distance=0.65,
                    navigation_min_obstacle_distance=0.55,
                    navigation_path_min_obstacle_distance=0.30,
                    disable_navigation_path_filter=False)
        flow = self.make_flow(args)
        flow._map_safety = object()

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        def obj(name, x):
            item = type('ObjectStub', (), {})()
            item.name = name
            item.class_name = 'bottle'
            item.confidence = 0.9
            item.position = type('PositionStub', (), {})()
            item.position.x = x
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        unsafe = obj('unsafe-path', 1.0)
        safe = obj('safe-path', 2.0)
        candidates = [
            {'object': unsafe, 'range': 1.0},
            {'object': safe, 'range': 2.0},
        ]
        message = type('MessageStub', (), {})()
        start_pose = pose(0.0, 0.0)
        flow.current_navigation_pose = lambda: start_pose
        flow._is_duplicate_object = lambda *_args, **_kwargs: False
        flow._is_duplicate_target = lambda *_args, **_kwargs: False

        def navigation_entries(_message, entry, start_pose=None):
            self.assertIsNotNone(start_pose)
            name = entry['object'].name
            nav_pose = pose(entry['range'], 0.0)
            summary = {
                'target': {'name': name},
                'approach': {'distance_m': 0.65},
            }
            if name == 'unsafe-path':
                yield (nav_pose, nav_pose, nav_pose, summary, False, 0.80, 0.10)
            else:
                yield (nav_pose, nav_pose, nav_pose, summary, True, 0.80, 0.80)

        flow._navigation_entries = navigation_entries

        selected, metadata = flow.select_navigation_targets(
            message, candidates, 1)

        self.assertEqual('safe-path', selected[0]['summary']['target']['name'])
        self.assertEqual(1, len(metadata['rejected_navigation_candidates']))
        rejected = metadata['rejected_navigation_candidates'][0]
        self.assertEqual(0.10, rejected['path_nearest_map_obstacle_distance_m'])

    def test_select_navigation_targets_scores_all_safe_candidates(self):
        args = Args(selector='nearest',
                    target_dedup_distance=0.25,
                    approach_distance=0.65,
                    navigation_min_obstacle_distance=0.55,
                    navigation_path_min_obstacle_distance=0.30,
                    disable_navigation_path_filter=False)
        flow = self.make_flow(args)
        flow._map_safety = object()

        def pose(x, y):
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = x
            item.pose.position.y = y
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        target_pose = pose(4.0, 0.0)
        item = type('ObjectStub', (), {})()
        item.class_name = 'bottle'
        item.confidence = 0.9
        candidates = [{'object': item, 'range': 4.0}]
        message = type('MessageStub', (), {})()
        flow.current_navigation_pose = lambda: pose(0.0, 0.0)
        flow._is_duplicate_object = lambda *_args, **_kwargs: False
        flow._is_duplicate_target = lambda *_args, **_kwargs: False

        def summary(name, goal_x, approach_distance, path_length):
            return {
                'target': {
                    'name': name,
                    'navigation_position': {
                        'frame_id': 'map', 'x': target_pose.pose.position.x,
                        'y': target_pose.pose.position.y,
                        'z': target_pose.pose.position.z,
                    },
                },
                'approach': {
                    'distance_m': approach_distance,
                    'candidate_approach_distance_m': approach_distance,
                    'navigation_goal': {
                        'position': {'x': goal_x, 'y': 0.0, 'z': 0.0},
                    },
                    'nearest_map_obstacle_distance_m': 0.80,
                    'path_nearest_map_obstacle_distance_m': 0.40,
                    'routed_path_length_m': path_length,
                },
            }

        def navigation_entries(_message, entry, start_pose=None):
            self.assertIsNotNone(start_pose)
            far_nav_pose = pose(2.4, 0.0)
            near_nav_pose = pose(3.35, 0.0)
            yield (far_nav_pose, far_nav_pose, target_pose,
                   summary('far-doorway', 2.4, 1.60, 2.0), True, 0.80, 0.40, 2.0)
            yield (near_nav_pose, near_nav_pose, target_pose,
                   summary('near-in-room', 3.35, 0.65, 2.5), True, 0.70, 0.35, 2.5)

        flow._navigation_entries = navigation_entries

        selected, _metadata = flow.select_navigation_targets(
            message, candidates, 1)

        self.assertEqual('near-in-room', selected[0]['summary']['target']['name'])
        self.assertAlmostEqual(3.35,
                               selected[0]['navigation_pose'].pose.position.x)


class CompetitionAuditTest(unittest.TestCase):
    def test_competition_audit_passes_valid_four_room_flow(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    pickup_action='dry-run',
                    dropoff_action='dry-run',
                    final_exit='auto')
        summary = {
            'status': 'ROOM_PATROL_COMPLETE',
            'requested_targets': 1,
            'visited_targets': 1,
            'final_exit': {'status': 'EXIT_REACHED'},
            'targets': [{
                'status': 'DROPOFF_REACHED',
                'target': {'class_name': 'bottle'},
                'pre_pickup_dropoff': {
                    'status': 'DROPOFF_CONFIRMED',
                    'source': 'current_perception',
                },
                'pickup_action': {
                    'status': 'PICKUP_DRY_RUN_COMPLETE',
                    'hardware_commanded': False,
                },
                'dropoff': {
                    'status': 'DROPOFF_REACHED',
                    'dropoff_action': {
                        'status': 'DROPOFF_DRY_RUN_COMPLETE',
                        'hardware_commanded': False,
                    },
                },
            }],
        }

        audit = build_competition_flow_audit(args, summary)

        self.assertEqual('COMPETITION_AUDIT_PASSED', audit['status'])
        self.assertTrue(audit['competition_flow_required'])
        self.assertTrue(all(item['status'] != 'FAIL'
                            for item in audit['checks']))

    def test_competition_audit_rejects_partial_flow_with_exit_record(self):
        args = Args(visit_count=3,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    pickup_action='dry-run',
                    dropoff_action='dry-run',
                    garbage_competition_mode=True,
                    garbage_quota=3,
                    final_exit='auto')
        summary = {
            'status': 'ROOM_PATROL_PARTIAL',
            'requested_targets': 3,
            'visited_targets': 1,
            'completed_dropoffs': 0,
            'final_exit': {'status': 'EXIT_REACHED'},
            'targets': [{
                'status': 'DROPOFF_PRECHECK_FAILED',
                'target': {'class_name': 'bottle'},
            }],
            'completed_rooms': [],
        }

        audit = build_competition_flow_audit(args, summary)
        checks = {item['name']: item for item in audit['checks']}

        self.assertEqual('COMPETITION_AUDIT_FAILED', audit['status'])
        self.assertEqual('FAIL',
                         checks['requested_targets_completed']['status'])
        self.assertEqual('OK',
                         checks['final_exit_recorded_after_completed_dropoffs']['status'])
        self.assertEqual('FAIL',
                         checks['garbage_competition_quota_completed']['status'])

    def test_competition_audit_rejects_trash_bin_as_pickup_target(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'trash_bin'],
                    dropoff_classes=['trash_bin'],
                    final_exit='none')
        summary = {
            'status': 'ROOM_PATROL_COMPLETE',
            'requested_targets': 1,
            'visited_targets': 1,
            'targets': [{
                'status': 'DROPOFF_REACHED',
                'target': {'class_name': 'trash_bin'},
                'pre_pickup_dropoff': {'status': 'DROPOFF_CONFIRMED'},
                'pickup_action': {'hardware_commanded': False},
                'dropoff': {
                    'status': 'DROPOFF_REACHED',
                    'dropoff_action': {'hardware_commanded': False},
                },
            }],
        }

        audit = build_competition_flow_audit(args, summary)
        checks = {item['name']: item for item in audit['checks']}

        self.assertEqual('COMPETITION_AUDIT_FAILED', audit['status'])
        self.assertEqual('FAIL',
                         checks['pickup_targets_exclude_dropoff']['status'])

    def test_competition_audit_rejects_missing_dropoff_confirmation(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='none')
        summary = {
            'status': 'ROOM_PATROL_COMPLETE',
            'requested_targets': 1,
            'visited_targets': 1,
            'targets': [{
                'status': 'DROPOFF_REACHED',
                'target': {'class_name': 'bottle'},
                'pickup_action': {'hardware_commanded': False},
                'dropoff': {
                    'status': 'DROPOFF_REACHED',
                    'dropoff_action': {'hardware_commanded': False},
                },
            }],
        }

        audit = build_competition_flow_audit(args, summary)
        checks = {item['name']: item for item in audit['checks']}

        self.assertEqual('COMPETITION_AUDIT_FAILED', audit['status'])
        self.assertEqual('FAIL',
                         checks['trash_bin_confirmed_before_each_pickup']['status'])



class RoomPatrolNavigationTest(unittest.TestCase):
    def make_flow(self, args):
        flow = TaskFlow.__new__(TaskFlow)
        flow.args = args
        flow.min_targets = 1
        flow._map_safety = None
        return flow

    def pose(self, x, y):
        item = type('PoseStampedStub', (), {})()
        item.header = type('HeaderStub', (), {'frame_id': 'map'})()
        item.pose = type('PoseStub', (), {})()
        item.pose.position = type('PositionStub', (), {})()
        item.pose.orientation = type('OrientationStub', (), {})()
        item.pose.position.x = x
        item.pose.position.y = y
        item.pose.position.z = 0.0
        item.pose.orientation.x = 0.0
        item.pose.orientation.y = 0.0
        item.pose.orientation.z = 0.0
        item.pose.orientation.w = 1.0
        return item

    def obj(self, class_name, confidence=0.9, x=1.0, y=0.0):
        item = type('ObjectStub', (), {})()
        item.class_name = class_name
        item.confidence = confidence
        item.position = type('PositionStub', (), {})()
        item.position.x = x
        item.position.y = y
        item.position.z = 0.0
        item.bbox_x = 0.0
        item.bbox_y = 0.0
        item.bbox_width = 1.0
        item.bbox_height = 1.0
        return item

    def message(self, *classes):
        msg = type('MessageStub', (), {})()
        msg.header = type('HeaderStub', (), {'frame_id': 'base_link'})()
        msg.objects = [self.obj(class_name) for class_name in classes]
        return msg

    def map_message(self, *items):
        msg = type('MessageStub', (), {})()
        msg.header = type('HeaderStub', (), {'frame_id': 'map'})()
        msg.objects = [self.obj(class_name, x=x, y=y)
                       for class_name, x, y in items]
        return msg

    def test_room_patrol_waypoints_follow_competition_order(self):
        args = Args(scenario='four_paper_ball', room_patrol='on')

        waypoints = room_patrol_waypoints(args)

        self.assertEqual(['living_room', 'kitchen', 'bedroom', 'bedroom', 'dining_room'],
                         [item['room'] for item in waypoints])
        self.assertEqual(['bottle'], waypoints[0]['classes'])
        self.assertAlmostEqual(8.39526, waypoints[0]['x'])
        self.assertAlmostEqual(1.52033, waypoints[0]['y'])
        self.assertEqual(4, len(waypoints[0]['scan_offsets']))
        self.assertAlmostEqual(math.radians(180.0), waypoints[0]['scan_offsets'][-1])
        self.assertAlmostEqual(7.02370, waypoints[1]['x'])
        self.assertAlmostEqual(6.98012, waypoints[1]['y'])
        self.assertAlmostEqual(math.radians(260.0), waypoints[1]['scan_offsets'][-1])
        self.assertEqual(['paper_ball'], waypoints[2]['classes'])
        self.assertAlmostEqual(2.80257, waypoints[2]['x'])
        self.assertAlmostEqual(4.03038, waypoints[2]['y'])
        self.assertEqual([], waypoints[2]['scan_offsets'])
        self.assertEqual('bedroom', waypoints[3]['room'])
        self.assertAlmostEqual(1.02032, waypoints[3]['x'])
        self.assertEqual(['box'], waypoints[-1]['classes'])
        self.assertAlmostEqual(2.32366, waypoints[-1]['x'])
        self.assertAlmostEqual(6.20868, waypoints[-1]['y'])
        self.assertEqual(4, len(waypoints[-1]['scan_offsets']))

    def test_room_patrol_marks_room_complete_only_after_dropoff_and_exits(self):
        args = Args(visit_count=4,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        messages = iter([
            self.map_message(('bottle', 5.55, 3.10)),
            self.map_message(('bottle', 6.80, 6.25)),
            self.map_message(('paper_ball', 1.65, 3.20)),
            self.map_message(('box', 3.432, 7.594)),
        ])
        selected_classes = []

        def wait_for_target_with_timeout(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return next(messages), ['candidate']

        def candidate_objects(message):
            return [{'object': item, 'range': 1.0} for item in message.objects]

        def select_navigation_targets(message, candidates, count, existing=None):
            self.assertEqual(1, count)
            selected_classes.append(candidates[0]['object'].class_name)
            index = len(selected_classes)
            return ([{
                'navigation_pose': self.pose(float(index), 0.0),
                'target_pose': self.pose(float(index) + 0.2, 0.0),
                'summary': {
                    'target': {'class_name': candidates[0]['object'].class_name},
                    'approach': {},
                },
            }], {})

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = candidate_objects
        flow.select_navigation_targets = select_navigation_targets
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': self.pose(9.0, 0.0),
            'target_pose': self.pose(9.2, 0.0),
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_COMPLETE', summary['status'])
        self.assertEqual(['living_room', 'kitchen', 'bedroom', 'dining_room'],
                         summary['completed_rooms'])
        self.assertEqual(['bottle', 'bottle', 'paper_ball', 'box'], selected_classes)
        self.assertEqual(4, summary['visited_targets'])
        self.assertTrue(all(item['status'] == 'ROOM_COMPLETE'
                            for item in summary['room_waypoints']))
        self.assertEqual('EXIT_REACHED', summary['final_exit']['status'])
        self.assertEqual('COMPETITION_AUDIT_PASSED',
                         summary['competition_audit']['status'])
        self.assertAlmostEqual(0.190553, sent_poses[-1].pose.position.x)
        self.assertAlmostEqual(7.89226, sent_poses[-1].pose.position.y)
        self.assertTrue(all(
            item['pickup_action']['status'] == 'PICKUP_DRY_RUN_COMPLETE'
            for item in summary['targets']))
        self.assertTrue(all(
            item['dropoff']['dropoff_action']['status'] == 'DROPOFF_DRY_RUN_COMPLETE'
            for item in summary['targets']))

    def test_garbage_competition_mode_stops_after_three_dropoffs_and_exits(self):
        args = Args(visit_count=4,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    garbage_competition_mode=True,
                    garbage_quota=3,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        messages = iter([
            self.map_message(('bottle', 5.55, 3.10)),
            self.map_message(('bottle', 6.80, 6.25)),
            self.map_message(('paper_ball', 1.65, 3.20)),
        ])
        selected_classes = []

        def wait_for_target_with_timeout(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return next(messages), ['candidate']

        def candidate_objects(message):
            return [{'object': item, 'range': 1.0} for item in message.objects]

        def select_navigation_targets(message, candidates, count, existing=None):
            self.assertEqual(1, count)
            selected_classes.append(candidates[0]['object'].class_name)
            index = len(selected_classes)
            return ([{
                'navigation_pose': self.pose(float(index), 0.0),
                'target_pose': self.pose(float(index) + 0.2, 0.0),
                'summary': {
                    'target': {'class_name': candidates[0]['object'].class_name},
                    'approach': {},
                },
            }], {})

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = candidate_objects
        flow.select_navigation_targets = select_navigation_targets
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': self.pose(9.0, 0.0),
            'target_pose': self.pose(9.2, 0.0),
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_COMPLETE', summary['status'])
        self.assertTrue(summary['garbage_competition_mode'])
        self.assertEqual(3, summary['garbage_quota'])
        self.assertEqual(3, summary['requested_targets'])
        self.assertEqual(3, summary['visited_targets'])
        self.assertEqual(3, summary['completed_dropoffs'])
        self.assertEqual(['living_room', 'kitchen', 'bedroom'],
                         summary['completed_rooms'])
        self.assertEqual(['bottle', 'bottle', 'paper_ball'], selected_classes)
        self.assertEqual(['dining_room'],
                         [item['room'] for item in summary['skipped_rooms_after_quota']])
        self.assertNotIn('dining_room',
                         [item['room'] for item in summary['room_waypoints']])
        self.assertEqual('EXIT_REACHED', summary['final_exit']['status'])
        self.assertEqual('COMPETITION_AUDIT_PASSED',
                         summary['competition_audit']['status'])
        self.assertAlmostEqual(0.190553, sent_poses[-1].pose.position.x)
        self.assertAlmostEqual(7.89226, sent_poses[-1].pose.position.y)

    def test_garbage_competition_dropoff_confirmation_failure_stops_before_pickup(self):
        args = Args(visit_count=4,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    garbage_competition_mode=True,
                    garbage_quota=3,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        flow.wait_for_target_with_timeout = lambda _timeout, require_fresh=False: (
            self.map_message(('bottle', 5.55, 3.10)), ['candidate'])
        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]
        flow.select_navigation_targets = lambda _message, candidates, _count, existing=None: ([{
            'navigation_pose': self.pose(1.0, 0.0),
            'target_pose': self.pose(1.2, 0.0),
            'summary': {
                'target': {'class_name': candidates[0]['object'].class_name},
                'approach': {},
            },
        }], {})

        def fail_dropoff_confirmation(_message):
            raise RuntimeError('trash bin not confirmed')

        flow.confirm_dropoff_before_pickup = fail_dropoff_confirmation
        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_PARTIAL', summary['status'])
        self.assertEqual(1, summary['visited_targets'])
        self.assertEqual(0, summary['completed_dropoffs'])
        self.assertEqual([], summary['completed_rooms'])
        self.assertEqual('DROPOFF_PRECHECK_FAILED', summary['targets'][0]['status'])
        self.assertEqual('ROOM_DROPOFF_PRECHECK_FAILED',
                         summary['room_waypoints'][0]['status'])
        self.assertNotIn('skipped_rooms_after_quota', summary)
        self.assertNotIn('final_exit', summary)
        self.assertEqual('COMPETITION_AUDIT_FAILED',
                         summary['competition_audit']['status'])
        self.assertEqual([], sent_poses)

    def test_garbage_competition_dropoff_failure_does_not_count_quota_or_skip_rooms(self):
        args = Args(visit_count=4,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    garbage_competition_mode=True,
                    garbage_quota=3,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        messages = iter([
            self.map_message(('bottle', 5.55, 3.10)),
            self.map_message(('bottle', 6.80, 6.25)),
        ])
        selected_classes = []

        def wait_for_target_with_timeout(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return next(messages), ['candidate']

        def candidate_objects(message):
            return [{'object': item, 'range': 1.0} for item in message.objects]

        def select_navigation_targets(message, candidates, count, existing=None):
            self.assertEqual(1, count)
            selected_classes.append(candidates[0]['object'].class_name)
            index = len(selected_classes)
            return ([{
                'navigation_pose': self.pose(float(index), 0.0),
                'target_pose': self.pose(float(index) + 0.2, 0.0),
                'summary': {
                    'target': {'class_name': candidates[0]['object'].class_name},
                    'approach': {},
                },
            }], {})

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = candidate_objects
        flow.select_navigation_targets = select_navigation_targets
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': self.pose(9.0, 0.0),
            'target_pose': self.pose(9.2, 0.0),
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            def fake_send_move_base_goal(_action_name, sent_pose, _timeout):
                sent_poses.append(sent_pose)
                if len(sent_poses) == 4:
                    raise RuntimeError('dropoff blocked')

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_PARTIAL', summary['status'])
        self.assertEqual(2, summary['visited_targets'])
        self.assertEqual(1, summary['completed_dropoffs'])
        self.assertEqual(['living_room'], summary['completed_rooms'])
        self.assertEqual(['bottle', 'bottle'], selected_classes)
        self.assertEqual('DROPOFF_FAILED', summary['targets'][1]['status'])
        self.assertEqual('ROOM_DROPOFF_FAILED', summary['room_waypoints'][1]['status'])
        self.assertNotIn('skipped_rooms_after_quota', summary)
        self.assertNotIn('final_exit', summary)

    def test_room_patrol_scans_when_direct_detection_has_wrong_class(self):
        args = Args(scenario='four_paper_ball',
                    room_patrol='on',
                    recovery_scan_timeout=0.25,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        waypoint = room_patrol_waypoints(args)[1]
        room_summary = {}
        waits = []
        sent_scans = []

        def obj(class_name):
            item = type('ObjectStub', (), {})()
            item.class_name = class_name
            item.confidence = 0.99
            item.position = type('PositionStub', (), {})()
            item.position.x = 1.0
            item.position.y = 0.0
            item.position.z = 0.0
            return item

        def message(class_name):
            item = type('MessageStub', (), {})()
            item.objects = [obj(class_name)]
            return item

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            waits.append((timeout, require_fresh))
            if len(waits) == 1:
                return self.map_message(('paper_ball', 1.65, 3.20)), []
            return self.map_message(('bottle', 6.80, 6.25)), []

        def pose():
            item = type('PoseStampedStub', (), {})()
            item.header = type('HeaderStub', (), {'frame_id': 'map'})()
            item.pose = type('PoseStub', (), {})()
            item.pose.position = type('PositionStub', (), {})()
            item.pose.orientation = type('OrientationStub', (), {})()
            item.pose.position.x = waypoint['x']
            item.pose.position.y = waypoint['y']
            item.pose.position.z = 0.0
            item.pose.orientation.x = 0.0
            item.pose.orientation.y = 0.0
            item.pose.orientation.z = 0.0
            item.pose.orientation.w = 1.0
            return item

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout

        import run_robocup_task_flow
        original_send_recovery_scan_goal = run_robocup_task_flow.send_recovery_scan_goal
        try:
            def fake_send_recovery_scan_goal(_args, sent_pose, yaw_offset):
                sent_scans.append(yaw_offset)
                return sent_pose

            run_robocup_task_flow.send_recovery_scan_goal = fake_send_recovery_scan_goal
            found_message, _candidates, room_candidates = wait_for_room_patrol_target(
                args, flow, waypoint, room_summary, pose())
        finally:
            run_robocup_task_flow.send_recovery_scan_goal = original_send_recovery_scan_goal

        self.assertEqual('bottle', found_message.objects[0].class_name)
        self.assertEqual(1, len(room_candidates))
        self.assertEqual('room_patrol_scan', room_summary['detection_source'])
        self.assertEqual(1, len(sent_scans))
        self.assertEqual(2, len(waits))

    def test_room_patrol_scan_samples_until_room_candidate_appears(self):
        args = Args(scenario='four_paper_ball',
                    room_patrol='on',
                    recovery_scan_timeout=0.25,
                    move_base_action='/move_base',
                    navigation_timeout=1.0)
        flow = self.make_flow(args)
        waypoint = room_patrol_waypoints(args)[1]
        room_summary = {}
        waits = []
        sent_scans = []
        scan_messages = iter([
            self.map_message(('paper_ball', 1.65, 3.20)),
            self.map_message(('bottle', 8.10, 7.20)),
        ])

        def wait_for_target_with_timeout(timeout, require_fresh=False):
            waits.append((timeout, require_fresh))
            if len(waits) == 1:
                return self.map_message(('paper_ball', 1.65, 3.20)), []
            return next(scan_messages), []

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]

        import run_robocup_task_flow
        original_send_recovery_scan_goal = run_robocup_task_flow.send_recovery_scan_goal
        try:
            def fake_send_recovery_scan_goal(_args, sent_pose, yaw_offset):
                sent_scans.append(yaw_offset)
                return sent_pose

            run_robocup_task_flow.send_recovery_scan_goal = fake_send_recovery_scan_goal
            found_message, _candidates, room_candidates = wait_for_room_patrol_target(
                args, flow, waypoint, room_summary, self.pose(waypoint['x'], waypoint['y']))
        finally:
            run_robocup_task_flow.send_recovery_scan_goal = original_send_recovery_scan_goal

        self.assertEqual('bottle', found_message.objects[0].class_name)
        self.assertEqual(1, len(room_candidates))
        self.assertEqual('room_patrol_scan', room_summary['detection_source'])
        self.assertEqual(1, len(sent_scans))
        self.assertEqual(3, len(waits))
        self.assertEqual(2, room_summary['scans'][0]['sample_count'])

    def test_room_patrol_rejects_trash_bin_as_pickup_candidate(self):
        args = Args(scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    dropoff_classes=['trash_bin'],
                    room_patrol='on')
        flow = self.make_flow(args)
        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]

        trash_bin = self.map_message(('trash_bin', 5.55, 3.10))
        waypoint = room_patrol_waypoints(args)[0]

        self.assertEqual([], room_patrol_candidates(flow, trash_bin, waypoint))

    def test_room_patrol_rejects_wrong_room_paper_ball_detection(self):
        args = Args(scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    room_patrol='on')
        flow = self.make_flow(args)
        waypoint = room_patrol_waypoints(args)[2]

        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]

        wrong_room = self.map_message(('paper_ball', 8.037, 6.802))
        bedroom = self.map_message(('paper_ball', 1.65, 3.20))

        self.assertEqual([], room_patrol_candidates(flow, wrong_room, waypoint))
        self.assertEqual('paper_ball',
                         room_patrol_candidates(flow, bedroom, waypoint)[0]['object'].class_name)


    def test_room_patrol_target_loss_does_not_reuse_stale_detection_or_exit(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    room_patrol='on',
                    recovery_scan_timeout=0.1,
                    navigation_timeout=0.1,
                    final_exit='auto')
        flow = self.make_flow(args)
        stale_message = self.map_message(('bottle', 1.65, 3.20))
        sent_goals = []

        def no_fresh_target(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return stale_message, []

        flow.wait_for_target_with_timeout = no_fresh_target
        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, pose, _timeout: sent_goals.append(pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_PARTIAL', summary['status'])
        self.assertEqual(0, summary['visited_targets'])
        self.assertEqual(0, summary['completed_dropoffs'])
        self.assertEqual([], summary['completed_rooms'])
        self.assertEqual([], summary['targets'])
        self.assertNotIn('final_exit', summary)
        self.assertNotIn('skipped_rooms_after_quota', summary)
        self.assertTrue(summary['room_waypoints'])
        self.assertTrue(all(item['status'] == 'ROOM_PATROL_FAILED'
                            for item in summary['room_waypoints']))
        self.assertTrue(sent_goals)

    def test_room_patrol_unreachable_candidate_is_not_picked_or_counted(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    room_patrol='on',
                    recovery_scan_timeout=0.1,
                    navigation_timeout=0.1,
                    final_exit='auto')
        flow = self.make_flow(args)
        message = self.map_message(('bottle', 5.55, 3.10))
        sent_goals = []
        flow.wait_for_target_with_timeout = lambda _timeout, require_fresh=False: (
            message, ['unreachable-candidate'])
        flow._candidate_objects = lambda current: [
            {'object': item, 'range': 1.0} for item in current.objects]

        def reject_candidate(_message, _candidates, _count, existing=None):
            raise RuntimeError('No navigation candidate satisfies 0.55 m endpoint/path map safety')

        flow.select_navigation_targets = reject_candidate
        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, pose, _timeout: sent_goals.append(pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_PARTIAL', summary['status'])
        self.assertEqual(0, summary['visited_targets'])
        self.assertEqual(0, summary['completed_dropoffs'])
        self.assertEqual([], summary['targets'])
        self.assertEqual([], summary['completed_rooms'])
        self.assertNotIn('final_exit', summary)
        self.assertNotIn('skipped_rooms_after_quota', summary)
        self.assertTrue(summary['room_waypoints'])
        self.assertTrue(all(item['status'] == 'ROOM_PATROL_FAILED'
                            for item in summary['room_waypoints']))
        self.assertTrue(sent_goals)

    def test_room_patrol_navigation_failure_does_not_reuse_old_target_pose(self):
        args = Args(visit_count=1,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    final_exit='none',
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        messages = iter([
            self.map_message(('bottle', 5.55, 3.10)),
            self.map_message(('bottle', 6.80, 6.25)),
        ])
        selected_poses = []

        def wait_for_target_with_timeout(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return next(messages), ['candidate']

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = lambda message: [
            {'object': item, 'range': 1.0} for item in message.objects]

        def select_navigation_targets(_message, candidates, _count, existing=None):
            index = len(selected_poses) + 1
            pose = self.pose(float(index), 0.0)
            selected_poses.append(pose)
            return ([{
                'navigation_pose': pose,
                'target_pose': self.pose(float(index) + 0.2, 0.0),
                'summary': {
                    'target': {'class_name': candidates[0]['object'].class_name},
                    'approach': {},
                },
            }], {})

        flow.select_navigation_targets = select_navigation_targets
        flow.confirm_dropoff_before_pickup = lambda _message: {
            'status': 'DROPOFF_CONFIRMED', 'source': 'unit_test'}
        flow.select_dropoff_navigation_target = lambda _message: {
            'navigation_pose': self.pose(9.0, 0.0),
            'target_pose': self.pose(9.2, 0.0),
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            def fake_send_move_base_goal(_action_name, sent_pose, _timeout):
                sent_poses.append(sent_pose)
                if len(sent_poses) == 1:
                    raise RuntimeError('pickup route blocked')

            run_robocup_task_flow.send_move_base_goal = fake_send_move_base_goal
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_COMPLETE', summary['status'])
        self.assertEqual(1, summary['visited_targets'])
        self.assertEqual('DROPOFF_REACHED', summary['targets'][0]['status'])
        self.assertEqual('NAVIGATION_FAILED',
                         summary['navigation_failures'][0]['status'])
        self.assertEqual('ROOM_TARGET_NAVIGATION_FAILED',
                         summary['room_waypoints'][0]['status'])
        self.assertEqual(2, len(selected_poses))
        self.assertAlmostEqual(1.0, selected_poses[0].pose.position.x)
        self.assertAlmostEqual(2.0, selected_poses[1].pose.position.x)
        self.assertAlmostEqual(1.0, sent_poses[0].pose.position.x)
        self.assertAlmostEqual(2.0, sent_poses[1].pose.position.x)
        self.assertAlmostEqual(9.0, sent_poses[2].pose.position.x)
        self.assertNotAlmostEqual(1.0, sent_poses[1].pose.position.x)

    def test_room_patrol_does_not_complete_room_when_dropoff_fails(self):
        args = Args(visit_count=4,
                    scenario='four_paper_ball',
                    target_classes=['bottle', 'paper_ball', 'box'],
                    final_exit='auto',
                    room_patrol='on',
                    selector='nearest',
                    target_dedup_distance=0.25,
                    navigation_min_obstacle_distance=0.55)
        flow = self.make_flow(args)
        messages = iter([
            self.map_message(('bottle', 5.55, 3.10)),
            self.map_message(('bottle', 6.80, 6.25)),
            self.map_message(('paper_ball', 1.65, 3.20)),
            self.map_message(('paper_ball', 1.65, 3.20)),
            self.map_message(('box', 3.432, 7.594)),
        ])
        selected_classes = []

        def wait_for_target_with_timeout(_timeout, require_fresh=False):
            self.assertTrue(require_fresh)
            return next(messages), ['candidate']

        def candidate_objects(message):
            return [{'object': item, 'range': 1.0} for item in message.objects]

        def select_navigation_targets(message, candidates, count, existing=None):
            self.assertEqual(1, count)
            selected_classes.append(candidates[0]['object'].class_name)
            index = len(selected_classes)
            return ([{
                'navigation_pose': self.pose(float(index), 0.0),
                'target_pose': self.pose(float(index) + 0.2, 0.0),
                'summary': {
                    'target': {'class_name': candidates[0]['object'].class_name},
                    'approach': {},
                },
            }], {})

        flow.wait_for_target_with_timeout = wait_for_target_with_timeout
        flow._candidate_objects = candidate_objects
        flow.select_navigation_targets = select_navigation_targets
        flow.select_dropoff_navigation_targets = lambda _message: [{
            'navigation_pose': self.pose(9.0, 0.0),
            'target_pose': self.pose(9.2, 0.0),
            'summary': {'target': {'class_name': 'trash_bin'}, 'approach': {}},
        }]

        import run_robocup_task_flow
        original_send_move_base_goal = run_robocup_task_flow.send_move_base_goal
        sent_poses = []
        try:
            run_robocup_task_flow.send_move_base_goal = (
                lambda _action_name, sent_pose, _timeout: sent_poses.append(sent_pose))
            summary = run_room_patrol_navigation(args, flow)
        finally:
            run_robocup_task_flow.send_move_base_goal = original_send_move_base_goal

        self.assertEqual('ROOM_PATROL_COMPLETE', summary['status'])
        self.assertEqual(['living_room', 'kitchen', 'bedroom', 'dining_room'],
                         summary['completed_rooms'])
        self.assertEqual(['bottle', 'bottle', 'paper_ball', 'box'], selected_classes)
        self.assertEqual(['wp1_living_room_left_180_scan',
                          'wp2_kitchen_260_scan',
                          'wp3_bedroom_front_check',
                          'wp5_dining_room_360_scan'],
                         [item['name'] for item in summary['room_waypoints']])
        self.assertNotIn('wp4_bedroom_front_backup',
                         [item['name'] for item in summary['room_waypoints']])
        bedroom_visit = [item for item in summary['room_waypoints']
                         if item['room'] == 'bedroom'][0]
        self.assertEqual('wp3_bedroom_front_check', bedroom_visit['name'])
        self.assertTrue(bedroom_visit.get('pre_waypoint_skip'))
        self.assertEqual('room_pre_waypoint_direct',
                         bedroom_visit.get('detection_source'))
        self.assertEqual('EXIT_REACHED', summary['final_exit']['status'])
        self.assertEqual(10, len(sent_poses))


if __name__ == '__main__':
    unittest.main()
