#!/usr/bin/env python3
"""Stabilize far paper_ball localizations in a fixed frame.

The RGB-D localizer publishes one 3D point per detection frame.  While the robot
is moving, navigation should not chase single-frame jitter.  This node keeps a
short window of map-frame observations, clusters nearby points, and publishes a
median target once the same object has been seen in several frames.
"""

from __future__ import print_function

import math
from collections import deque

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from garbage_localization.msg import GarbageObject, GarbageObjectArray


def _get_param(name, default):
    nested_name = '~stabilizer/{}'.format(name)
    flat_name = '~{}'.format(name)
    if rospy.has_param(nested_name):
        return rospy.get_param(nested_name)
    return rospy.get_param(flat_name, default)


def _param_list(name, default):
    value = _get_param(name, default)
    if isinstance(value, str):
        return [item.strip() for item in value.split(',') if item.strip()]
    return list(value)


def _median(values):
    ordered = sorted(values)
    if not ordered:
        return float('nan')
    middle = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[middle]
    return 0.5 * (ordered[middle - 1] + ordered[middle])


def _key_value(key, value):
    item = KeyValue()
    item.key = key
    item.value = str(value)
    return item


def _header_key(header):
    return '{}.{}:{}:{}'.format(
        header.stamp.secs, header.stamp.nsecs, header.seq, header.frame_id)


class Observation(object):
    __slots__ = (
        'stamp', 'header_key', 'class_name', 'confidence', 'x', 'y', 'z',
        'valid_point_count', 'median_depth', 'bbox_x', 'bbox_y', 'bbox_width',
        'bbox_height')

    def __init__(self, stamp, header_key, obj):
        self.stamp = stamp
        self.header_key = header_key
        self.class_name = obj.class_name
        self.confidence = float(obj.confidence)
        self.x = float(obj.position.x)
        self.y = float(obj.position.y)
        self.z = float(obj.position.z)
        self.valid_point_count = int(obj.valid_point_count)
        self.median_depth = float(obj.median_depth)
        self.bbox_x = float(obj.bbox_x)
        self.bbox_y = float(obj.bbox_y)
        self.bbox_width = float(obj.bbox_width)
        self.bbox_height = float(obj.bbox_height)

    def finite(self):
        return (math.isfinite(self.x) and math.isfinite(self.y) and
                math.isfinite(self.z) and math.isfinite(self.confidence))


class PaperBallMapStabilizer(object):
    def __init__(self):
        self.input_topic = _get_param('input_topic', '/garbage_localization/objects')
        self.output_topic = _get_param('output_topic', '/garbage_localization/stable_objects')
        self.diagnostics_topic = _get_param('diagnostics_topic', '/diagnostics')
        self.fixed_frame = _get_param('fixed_frame', 'map')
        self.target_classes = set(_param_list('target_classes', ['paper_ball']))
        self.min_confidence = float(_get_param('min_confidence', 0.12))
        self.min_valid_points = int(_get_param('min_valid_points', 5))
        self.min_observations = int(_get_param('min_observations', 3))
        self.window_seconds = float(_get_param('window_seconds', 4.0))
        self.cluster_radius = float(_get_param('cluster_radius', 0.35))
        self.publish_rate = float(_get_param('publish_rate', 5.0))

        self.observations = deque()
        self.last_input_stamp = rospy.Time(0)
        self.last_input_frame = ''
        self.frame_mismatch_count = 0
        self.filtered_count = 0

        self.publisher = rospy.Publisher(self.output_topic, GarbageObjectArray, queue_size=5)
        self.diagnostics_publisher = rospy.Publisher(
            self.diagnostics_topic, DiagnosticArray, queue_size=5)
        self.subscriber = rospy.Subscriber(
            self.input_topic, GarbageObjectArray, self._callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate, 0.1)),
                                 self._timer_callback)

        rospy.loginfo('paper_ball_map_stabilizer: input=%s output=%s frame=%s window=%.2fs min_obs=%d radius=%.2fm',
                      self.input_topic, self.output_topic, self.fixed_frame,
                      self.window_seconds, self.min_observations, self.cluster_radius)

    def _callback(self, message):
        self.last_input_stamp = message.header.stamp
        self.last_input_frame = message.header.frame_id
        if message.header.frame_id != self.fixed_frame:
            self.frame_mismatch_count += 1
            return

        source_header = (message.detection_header
                         if message.detection_header.stamp.to_sec() > 0.0
                         else message.header)
        source_key = _header_key(source_header)
        stamp = message.header.stamp if message.header.stamp else rospy.Time.now()
        for obj in message.objects:
            if obj.class_name not in self.target_classes:
                self.filtered_count += 1
                continue
            obs = Observation(stamp, source_key, obj)
            if (not obs.finite() or obs.confidence < self.min_confidence or
                    obs.valid_point_count < self.min_valid_points):
                self.filtered_count += 1
                continue
            self.observations.append(obs)
        self._prune(rospy.Time.now())

    def _prune(self, now):
        cutoff = now - rospy.Duration(self.window_seconds)
        while self.observations and self.observations[0].stamp < cutoff:
            self.observations.popleft()

    def _distance_xy(self, left, right):
        return math.hypot(left.x - right.x, left.y - right.y)

    def _cluster_observations(self):
        remaining = list(self.observations)
        clusters = []
        while remaining:
            seed = max(remaining, key=lambda obs: (obs.confidence, obs.valid_point_count))
            same_class = [obs for obs in remaining if obs.class_name == seed.class_name]
            cluster = [obs for obs in same_class if self._distance_xy(obs, seed) <= self.cluster_radius]
            if cluster:
                center_x = _median([obs.x for obs in cluster])
                center_y = _median([obs.y for obs in cluster])
                refined = [obs for obs in cluster
                           if math.hypot(obs.x - center_x, obs.y - center_y) <= self.cluster_radius]
                cluster = refined or cluster
            used_ids = set(id(obs) for obs in cluster)
            remaining = [obs for obs in remaining if id(obs) not in used_ids]

            distinct_headers = set(obs.header_key for obs in cluster)
            if len(cluster) >= self.min_observations and len(distinct_headers) >= self.min_observations:
                clusters.append(cluster)

        clusters.sort(key=lambda c: (len(c), _median([o.confidence for o in c])), reverse=True)
        return clusters

    def _object_from_cluster(self, cluster):
        best = max(cluster, key=lambda obs: (obs.confidence, obs.valid_point_count))
        obj = GarbageObject()
        obj.class_name = best.class_name
        obj.confidence = float(_median([obs.confidence for obs in cluster]))
        obj.position.x = float(_median([obs.x for obs in cluster]))
        obj.position.y = float(_median([obs.y for obs in cluster]))
        obj.position.z = float(_median([obs.z for obs in cluster]))
        obj.bbox_x = best.bbox_x
        obj.bbox_y = best.bbox_y
        obj.bbox_width = best.bbox_width
        obj.bbox_height = best.bbox_height
        obj.valid_point_count = sum(obs.valid_point_count for obs in cluster)
        obj.median_depth = float(_median([obs.median_depth for obs in cluster]))
        return obj

    def _timer_callback(self, _event):
        now = rospy.Time.now()
        self._prune(now)
        clusters = self._cluster_observations()

        output = GarbageObjectArray()
        output.header.stamp = now
        output.header.frame_id = self.fixed_frame
        output.detection_header.stamp = self.last_input_stamp
        output.detection_header.frame_id = self.last_input_frame
        output.detector_backend = 'garbage_localization+map_stabilized'
        output.objects = [self._object_from_cluster(cluster) for cluster in clusters]
        self.publisher.publish(output)
        self._publish_diagnostics(output, clusters)

    def _publish_diagnostics(self, output, clusters):
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        status = DiagnosticStatus()
        status.name = 'paper_ball_map_stabilizer/status'
        status.hardware_id = 'rgbd_map_stabilizer'
        status.level = DiagnosticStatus.OK if output.objects else DiagnosticStatus.WARN
        status.message = 'stable target OK' if output.objects else 'waiting for stable map-frame target'
        status.values = [
            _key_value('input_topic', self.input_topic),
            _key_value('output_topic', self.output_topic),
            _key_value('fixed_frame', self.fixed_frame),
            _key_value('observation_count', len(self.observations)),
            _key_value('stable_object_count', len(output.objects)),
            _key_value('cluster_count', len(clusters)),
            _key_value('min_observations', self.min_observations),
            _key_value('cluster_radius', self.cluster_radius),
            _key_value('frame_mismatch_count', self.frame_mismatch_count),
            _key_value('filtered_count', self.filtered_count),
            _key_value('last_input_frame', self.last_input_frame),
        ]
        array.status.append(status)
        self.diagnostics_publisher.publish(array)


def main():
    rospy.init_node('paper_ball_map_stabilizer')
    PaperBallMapStabilizer()
    rospy.spin()


if __name__ == '__main__':
    main()
