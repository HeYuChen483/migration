#!/usr/bin/env python3
"""One-shot Gazebo scene calibration for the four_paper_ball RoboCup sim."""

from __future__ import print_function

import argparse
import sys
import time

import rospy
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import GetModelState, SetModelState
from tf.transformations import quaternion_from_euler


def wait_for_model(get_state, model_name, timeout):
    deadline = time.monotonic() + timeout
    last_error = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        try:
            state = get_state(model_name, 'world')
        except rospy.ServiceException as exc:
            last_error = exc
            time.sleep(0.1)
            continue
        if state.success:
            return state
        time.sleep(0.1)
    if last_error is not None:
        raise RuntimeError('Timed out waiting for model {}: {}'.format(
            model_name, last_error))
    raise RuntimeError('Timed out waiting for model {}'.format(model_name))


def set_model_pose(set_state, model_name, x, y, z, yaw):
    quat = quaternion_from_euler(0.0, 0.0, yaw)
    state = ModelState()
    state.model_name = model_name
    state.reference_frame = 'world'
    state.pose.position.x = x
    state.pose.position.y = y
    state.pose.position.z = z
    state.pose.orientation.x = quat[0]
    state.pose.orientation.y = quat[1]
    state.pose.orientation.z = quat[2]
    state.pose.orientation.w = quat[3]
    response = set_state(state)
    if not response.success:
        raise RuntimeError('Failed to set model {}: {}'.format(
            model_name, response.status_message))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Stabilize task objects after Gazebo scene startup')
    parser.add_argument('--model', default='ground_red_bottle')
    parser.add_argument('--support-model', default='tea_table')
    parser.add_argument('--x', type=float, default=5.55)
    parser.add_argument('--y', type=float, default=3.10)
    parser.add_argument('--z', type=float, default=0.52)
    parser.add_argument('--yaw', type=float, default=0.0)
    parser.add_argument('--wait-timeout', type=float, default=20.0)
    parser.add_argument('--settle-time', type=float, default=0.5)
    parser.add_argument('--verify-timeout', type=float, default=3.0)
    parser.add_argument('--min-final-z', type=float, default=0.35)
    args, _unknown = parser.parse_known_args(argv)
    return args


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rospy.init_node('four_paper_ball_scene_calibrator', anonymous=False)
    rospy.wait_for_service('/gazebo/get_model_state', timeout=args.wait_timeout)
    rospy.wait_for_service('/gazebo/set_model_state', timeout=args.wait_timeout)
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    wait_for_model(get_state, args.support_model, args.wait_timeout)
    wait_for_model(get_state, args.model, args.wait_timeout)
    set_model_pose(set_state, args.model, args.x, args.y, args.z, args.yaw)
    if args.settle_time > 0.0:
        rospy.sleep(args.settle_time)

    deadline = time.monotonic() + args.verify_timeout
    final_state = None
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        final_state = wait_for_model(get_state, args.model, 0.5)
        if final_state.pose.position.z >= args.min_final_z:
            p = final_state.pose.position
            rospy.loginfo('Calibrated %s at x=%.3f y=%.3f z=%.3f',
                          args.model, p.x, p.y, p.z)
            return 0
        rospy.sleep(0.1)

    if final_state is None:
        final_state = get_state(args.model, 'world')
    p = final_state.pose.position
    rospy.logerr('Calibration left %s below expected height: x=%.3f y=%.3f z=%.3f',
                 args.model, p.x, p.y, p.z)
    return 1


if __name__ == '__main__':
    sys.exit(main())
