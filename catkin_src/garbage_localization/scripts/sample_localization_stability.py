#!/usr/bin/env python3
"""Sample localization output and report map-frame target stability."""

from __future__ import print_function

import argparse
import json
import math
import statistics

import rospy
from garbage_localization.msg import GarbageObjectArray


def median(values):
    return statistics.median(values) if values else None


def pstdev(values):
    return statistics.pstdev(values) if len(values) >= 2 else 0.0


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(math.ceil(fraction * len(ordered))) - 1))
    return ordered[index]


class Sampler(object):
    def __init__(self, topic, target_class, frame):
        self.topic = topic
        self.target_class = target_class
        self.frame = frame
        self.samples = []
        self.frame_mismatch = 0
        self.messages = 0
        self.subscriber = rospy.Subscriber(topic, GarbageObjectArray, self.callback, queue_size=20)

    def callback(self, message):
        self.messages += 1
        if self.frame and message.header.frame_id != self.frame:
            self.frame_mismatch += 1
            return
        for obj in message.objects:
            if obj.class_name != self.target_class:
                continue
            x = float(obj.position.x)
            y = float(obj.position.y)
            z = float(obj.position.z)
            if math.isfinite(x) and math.isfinite(y) and math.isfinite(z):
                self.samples.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'confidence': float(obj.confidence),
                    'valid_point_count': int(obj.valid_point_count),
                    'median_depth': float(obj.median_depth),
                })

    def summary(self):
        xs = [sample['x'] for sample in self.samples]
        ys = [sample['y'] for sample in self.samples]
        zs = [sample['z'] for sample in self.samples]
        center_x = median(xs)
        center_y = median(ys)
        radii = []
        if center_x is not None and center_y is not None:
            radii = [math.hypot(sample['x'] - center_x, sample['y'] - center_y)
                     for sample in self.samples]
        return {
            'topic': self.topic,
            'target_class': self.target_class,
            'frame': self.frame,
            'messages_seen': self.messages,
            'frame_mismatch_count': self.frame_mismatch,
            'sample_count': len(self.samples),
            'median_position': {'x': center_x, 'y': center_y, 'z': median(zs)},
            'std_m': {'x': pstdev(xs), 'y': pstdev(ys), 'z': pstdev(zs)},
            'xy_jitter_m': {
                'median': median(radii),
                'p95': percentile(radii, 0.95),
                'max': max(radii) if radii else None,
            },
            'median_confidence': median([sample['confidence'] for sample in self.samples]),
            'median_depth': median([sample['median_depth'] for sample in self.samples]),
            'median_valid_points': median([sample['valid_point_count'] for sample in self.samples]),
        }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/garbage_localization/stable_objects')
    parser.add_argument('--class-name', default='paper_ball')
    parser.add_argument('--frame', default='map')
    parser.add_argument('--seconds', type=float, default=8.0)
    args = parser.parse_args(argv)

    rospy.init_node('sample_localization_stability', anonymous=True)
    sampler = Sampler(args.topic, args.class_name, args.frame)
    rospy.sleep(args.seconds)
    print(json.dumps(sampler.summary(), indent=2, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
