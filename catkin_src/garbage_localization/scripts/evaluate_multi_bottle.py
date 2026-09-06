#!/usr/bin/env python3
"""Collect and evaluate Gazebo multi-bottle localization messages."""

from __future__ import print_function

import json
import math
import os
import sys
import time

SCRIPT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
if SCRIPT_DIRECTORY not in sys.path:
    sys.path.insert(0, SCRIPT_DIRECTORY)

import rospy
from gazebo_msgs.srv import GetModelState
from garbage_localization.msg import GarbageObjectArray
from vision_msgs.msg import DetectionArray

from multi_bottle_evaluation_utils import (
    acceptance_passed,
    apply_local_offset,
    is_valid_position,
    minimum_cost_assignment,
    output_count_category,
    position_in_reference_frame,
    summarize_errors,
)


class MultiBottleEvaluator(object):
    def __init__(self):
        self.model_names = rospy.get_param('~model_names',
                                           ['ground_red_bottle',
                                            'ground_red_bottle_second'])
        self.sample_count = rospy.get_param('~sample_count', 20)
        self.timeout_sec = rospy.get_param('~timeout_sec', 60.0)
        self.truth_z_offset = rospy.get_param('~truth_z_offset', 0.10)
        self.truth_local_offsets = rospy.get_param('~truth_local_offsets', {})
        self.reference_model = rospy.get_param('~reference_model', 'wpb_home')
        self.reference_frame = rospy.get_param('~reference_frame', 'base_link')
        self.maximum_median_error_m = rospy.get_param(
            '~maximum_median_error_m', 0.05)
        self.minimum_detection_complete_ratio = rospy.get_param(
            '~minimum_detection_complete_ratio', 0.0)
        self.minimum_localization_complete_ratio = rospy.get_param(
            '~minimum_localization_complete_ratio', 0.0)
        self.output_path = rospy.get_param('~output_path', '')
        self.detection_topic = rospy.get_param('~detection_topic',
                                               '/yolo_world/detections')
        self.objects_topic = rospy.get_param('~objects_topic',
                                             '/garbage_localization/objects')

        if len(self.model_names) < 1:
            raise ValueError('~model_names must contain at least one model')
        if len(set(self.model_names)) != len(self.model_names):
            raise ValueError('~model_names must contain unique Gazebo model names')
        if not self.reference_model or not self.reference_frame:
            raise ValueError('~reference_model and ~reference_frame must not be empty')
        if self.reference_model in self.model_names:
            raise ValueError('~reference_model must not also be a garbage model')
        if not isinstance(self.truth_local_offsets, dict):
            raise ValueError('~truth_local_offsets must be a model-to-XYZ mapping')
        if self.sample_count <= 0 or self.timeout_sec <= 0.0:
            raise ValueError('~sample_count and ~timeout_sec must be positive')
        if not (0.0 <= self.minimum_detection_complete_ratio <= 1.0):
            raise ValueError('~minimum_detection_complete_ratio must be between 0 and 1')
        if not (0.0 <= self.minimum_localization_complete_ratio <= 1.0):
            raise ValueError('~minimum_localization_complete_ratio must be between 0 and 1')

        self.detection_messages = 0
        self.detection_complete_messages = 0
        self.localization_messages = 0
        self.complete_output_messages = 0
        self.invalid_output_messages = 0
        self.incomplete_output_messages = 0
        self.undercomplete_output_messages = 0
        self.overcomplete_output_messages = 0
        self.errors_by_model = {name: [] for name in self.model_names}
        self._last_detection_stamp = None
        self._last_object_stamp = None

        rospy.Subscriber(self.detection_topic, DetectionArray,
                         self._detection_callback, queue_size=20)
        rospy.Subscriber(self.objects_topic, GarbageObjectArray,
                         self._objects_callback, queue_size=20)

    @staticmethod
    def _stamp_key(header):
        return (header.stamp.secs, header.stamp.nsecs)

    def _detection_callback(self, message):
        stamp = self._stamp_key(message.header)
        if stamp == self._last_detection_stamp:
            return
        self._last_detection_stamp = stamp
        self.detection_messages += 1
        if len(message.objects) >= len(self.model_names):
            self.detection_complete_messages += 1

    @staticmethod
    def _pose_components(pose):
        position = pose.position
        orientation = pose.orientation
        return ((position.x, position.y, position.z),
                (orientation.x, orientation.y, orientation.z, orientation.w))

    def _truth_positions(self):
        """Return Gazebo model centers in the localization output frame.

        Gazebo resolves an unscoped relative_entity_name such as base_link to
        whichever model link has that name.  Garbage URDFs commonly reuse
        base_link, so querying model state relative to base_link is ambiguous.
        Instead, query every pose in world and transform it explicitly through
        the uniquely named robot model pose.  In simulation, reference_model is
        the Gazebo counterpart of the ROS reference_frame; real-robot operation
        does not use this acceptance-only truth path.
        """
        rospy.wait_for_service('/gazebo/get_model_state', timeout=10.0)
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state',
                                             GetModelState)
        reference = get_model_state(self.reference_model, 'world')
        if not reference.success:
            raise RuntimeError(
                'Gazebo reference model query failed for {}: {}'.format(
                    self.reference_model, reference.status_message))
        reference_position, reference_orientation = self._pose_components(
            reference.pose)
        if not is_valid_position(reference_position):
            # A robot legitimately placed at the world origin has an all-zero
            # position; only finiteness matters for a reference pose.
            if not all(math.isfinite(value) for value in reference_position):
                raise RuntimeError(
                    'Gazebo reference model returned a non-finite position')
        if not all(math.isfinite(value) for value in reference_orientation):
            raise RuntimeError(
                'Gazebo reference model returned a non-finite orientation')

        positions = []
        for model_name in self.model_names:
            response = get_model_state(model_name, 'world')
            if not response.success:
                raise RuntimeError('Gazebo truth query failed for {}: {}'.format(
                    model_name, response.status_message))
            model_position, model_orientation = self._pose_components(
                response.pose)
            local_offset = self.truth_local_offsets.get(
                model_name, [0.0, 0.0, self.truth_z_offset])
            try:
                if len(local_offset) != 3:
                    raise ValueError('offset must contain XYZ')
                center_world = apply_local_offset(
                    model_position, model_orientation, tuple(local_offset))
                positions.append(position_in_reference_frame(
                    center_world, reference_position, reference_orientation))
            except (TypeError, ValueError) as error:
                raise RuntimeError(
                    'Invalid Gazebo pose or truth offset for {}: {}'.format(
                        model_name, error))
        return positions

    def _objects_callback(self, message):
        if self.complete_output_messages >= self.sample_count:
            return
        stamp = self._stamp_key(message.detection_header)
        if stamp == self._last_object_stamp:
            return
        self._last_object_stamp = stamp
        self.localization_messages += 1

        if message.header.frame_id != self.reference_frame:
            rospy.logwarn_throttle(
                5.0, 'Localization output frame %s does not match evaluation '
                'reference frame %s', message.header.frame_id,
                self.reference_frame)
            self.invalid_output_messages += 1
            return

        count_category = output_count_category(
            len(message.objects), len(self.model_names))
        if count_category != 'complete':
            self.incomplete_output_messages += 1
            if count_category == 'undercomplete':
                self.undercomplete_output_messages += 1
            else:
                self.overcomplete_output_messages += 1
            return

        estimated_positions = [(item.position.x, item.position.y, item.position.z)
                               for item in message.objects]
        if not all(is_valid_position(point) for point in estimated_positions):
            self.invalid_output_messages += 1
            return

        try:
            truth_positions = self._truth_positions()
        except (rospy.ROSException, rospy.ServiceException, RuntimeError) as error:
            rospy.logwarn('Could not query Gazebo truth: %s', error)
            self.invalid_output_messages += 1
            return

        assignment = minimum_cost_assignment(truth_positions, estimated_positions)
        if assignment is None:
            self.invalid_output_messages += 1
            return

        for model_name, (_, error) in zip(self.model_names, assignment):
            self.errors_by_model[model_name].append(error)
        self.complete_output_messages += 1

    def result(self, timed_out):
        per_model = {name: summarize_errors(errors)
                     for name, errors in self.errors_by_model.items()}
        detection_complete_ratio = (
            float(self.detection_complete_messages) / self.detection_messages
            if self.detection_messages else 0.0)
        localization_complete_ratio = (
            float(self.complete_output_messages) / self.localization_messages
            if self.localization_messages else 0.0)
        passed = acceptance_passed(
            timed_out, self.complete_output_messages, self.sample_count,
            self.invalid_output_messages, self.overcomplete_output_messages,
            per_model, self.maximum_median_error_m, detection_complete_ratio,
            self.minimum_detection_complete_ratio, localization_complete_ratio,
            self.minimum_localization_complete_ratio)
        return {
            'accepted': passed,
            'timed_out': timed_out,
            'model_names': self.model_names,
            'truth_reference': {
                'gazebo_model': self.reference_model,
                'output_frame': self.reference_frame,
                'method': 'world_pose_transform',
            },
            'sample_count_requested': self.sample_count,
            'detection_messages': self.detection_messages,
            'detection_complete_messages': self.detection_complete_messages,
            'detection_complete_ratio': detection_complete_ratio,
            'localization_messages': self.localization_messages,
            'complete_output_messages': self.complete_output_messages,
            'localization_complete_ratio': localization_complete_ratio,
            'incomplete_output_messages': self.incomplete_output_messages,
            'undercomplete_output_messages': self.undercomplete_output_messages,
            'overcomplete_output_messages': self.overcomplete_output_messages,
            'invalid_output_messages': self.invalid_output_messages,
            'maximum_median_error_m': self.maximum_median_error_m,
            'minimum_detection_complete_ratio': (
                self.minimum_detection_complete_ratio),
            'minimum_localization_complete_ratio': (
                self.minimum_localization_complete_ratio),
            'per_model': per_model,
        }

    def run(self):
        started = time.monotonic()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.complete_output_messages >= self.sample_count:
                return self.result(False)
            if time.monotonic() - started >= self.timeout_sec:
                return self.result(True)
            rate.sleep()
        return self.result(True)


def main():
    rospy.init_node('evaluate_multi_bottle', anonymous=True)
    try:
        evaluator = MultiBottleEvaluator()
        result = evaluator.run()
    except (ValueError, rospy.ROSException, rospy.ServiceException) as error:
        rospy.logerr('%s', error)
        return 2

    serialized = json.dumps(result, sort_keys=True, indent=2)
    output_path = evaluator.output_path
    if output_path:
        with open(output_path, 'w') as output_file:
            output_file.write(serialized + '\n')
    print(serialized)
    if result['accepted']:
        rospy.loginfo('Multi-bottle acceptance passed.')
        return 0
    rospy.logerr('Multi-bottle acceptance did not meet the configured criteria.')
    return 1


if __name__ == '__main__':
    sys.exit(main())
