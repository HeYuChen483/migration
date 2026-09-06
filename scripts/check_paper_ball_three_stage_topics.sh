#!/usr/bin/env bash
set -euo pipefail

script_args=("$@")
set --
source /opt/ros/noetic/setup.bash
source /home/hyc/catkin_wa/devel/setup.bash
set -- "${script_args[@]}"

python3 - "$@" <<'PY'
import argparse
import sys
import time

import rospy
from vision_msgs.msg import DetectionArray


TOPICS = [
    ("raw", "/yolo_world/paper_box_secondary_detections"),
    ("tracked", "/yolo_world/paper_box_secondary_tracked_detections"),
    ("voted", "/yolo_world/paper_box_secondary_rgbd_voted_detections"),
    ("enhanced", "/yolo_world/enhanced_detections"),
]


def center(obj):
    return obj.x + obj.width * 0.5, obj.y + obj.height * 0.5


def compact_object(obj):
    cx, cy = center(obj)
    return (
        f"id={obj.track_id:>4} conf={obj.confidence:0.3f} "
        f"center=({cx:6.1f},{cy:6.1f}) size=({obj.width:5.1f},{obj.height:5.1f})"
    )


def cluster_objects(objects, max_distance_px):
    clusters = []
    for obj in sorted(objects, key=lambda item: item.confidence, reverse=True):
        cx, cy = center(obj)
        best_cluster = None
        best_distance = None
        for cluster in clusters:
            dx = cx - cluster["cx"]
            dy = cy - cluster["cy"]
            distance = (dx * dx + dy * dy) ** 0.5
            if distance <= max_distance_px and (best_distance is None or distance < best_distance):
                best_cluster = cluster
                best_distance = distance
        if best_cluster is None:
            clusters.append({"cx": cx, "cy": cy, "count": 1, "best": obj})
            continue
        old_count = best_cluster["count"]
        new_count = old_count + 1
        best_cluster["cx"] = (best_cluster["cx"] * old_count + cx) / new_count
        best_cluster["cy"] = (best_cluster["cy"] * old_count + cy) / new_count
        best_cluster["count"] = new_count
        if obj.confidence > best_cluster["best"].confidence:
            best_cluster["best"] = obj
    clusters.sort(key=lambda cluster: cluster["cx"])
    return clusters


def compact_cluster(cluster):
    best = cluster["best"]
    bx, by = center(best)
    return (
        f"n={cluster['count']:>2} avg=({cluster['cx']:6.1f},{cluster['cy']:6.1f}) "
        f"best_conf={best.confidence:0.3f} best_center=({bx:6.1f},{by:6.1f})"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Watch paper_ball counts through raw, tracked/voted, and enhanced topics."
    )
    parser.add_argument("--hz", type=float, default=1.0)
    parser.add_argument("--samples", type=int, default=0, help="0 means run until Ctrl-C")
    parser.add_argument("--cluster-px", type=float, default=40.0)
    args = parser.parse_args(rospy.myargv(sys.argv)[1:])

    latest = {}

    def make_callback(name):
        def callback(msg):
            balls = [obj for obj in msg.objects if obj.class_name == "paper_ball"]
            balls.sort(key=lambda obj: (obj.x + obj.width * 0.5, obj.y + obj.height * 0.5))
            latest[name] = (time.time(), msg.header.seq, msg.backend, balls)

        return callback

    rospy.init_node("paper_ball_three_stage_topic_watch", anonymous=True)
    for name, topic in TOPICS:
        rospy.Subscriber(topic, DetectionArray, make_callback(name), queue_size=3)

    rate = rospy.Rate(max(args.hz, 0.2))
    sample = 0
    while not rospy.is_shutdown():
        sample += 1
        print(f"\n[{time.strftime('%H:%M:%S')}] paper_ball three-stage topic watch")
        for name, topic in TOPICS:
            if name not in latest:
                print(f"{name:>8} {topic}: no message yet")
                continue
            received_at, seq, backend, balls = latest[name]
            age = time.time() - received_at
            clusters = cluster_objects(balls, args.cluster_px)
            print(
                f"{name:>8} {topic}: count={len(balls)} clusters={len(clusters)} "
                f"seq={seq} age={age:0.2f}s backend={backend}"
            )
            for cluster in clusters:
                print(f"         cluster {compact_cluster(cluster)}")
            for obj in balls:
                print(f"         {compact_object(obj)}")
        sys.stdout.flush()
        if args.samples > 0 and sample >= args.samples:
            break
        rate.sleep()


if __name__ == "__main__":
    main()
PY
