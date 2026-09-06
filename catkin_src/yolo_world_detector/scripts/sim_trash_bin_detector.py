#!/usr/bin/env python3
"""Gazebo-only projected trash-bin detector for the RoboCup competition sim.

This node publishes a standard DetectionArray as class ``trash_bin`` for the
package-local living-room bin.  It projects the known Gazebo model into the RGB
camera using camera_info + TF, then keeps the existing depth localizer unchanged.
The older green-marker detector remains as a fallback only; the run-through no
longer depends on marker color being visible in the rendered image.
"""

import math

import cv2
import numpy as np
import rospy
import tf2_geometry_msgs  # noqa: F401 - registers PointStamped TF conversions
import tf2_ros
from cv_bridge import CvBridge, CvBridgeError
from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import DetectionArray, DetectionResult


class SimTrashBinDetector(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.class_name = rospy.get_param('~class_name', 'trash_bin')
        self.confidence = float(rospy.get_param('~confidence', 0.90))
        self.publish_empty = bool(rospy.get_param('~publish_empty', True))
        self.model_name = rospy.get_param('~model_name', 'living_room_trash_bin')
        self.model_reference_frame = rospy.get_param('~model_reference_frame', 'wpb_home')
        self.model_tf_frame = rospy.get_param('~model_tf_frame', 'base_link')
        self.camera_frame = rospy.get_param('~camera_frame', '')
        self.projected_min_width = float(rospy.get_param('~projected_min_width', 80.0))
        self.projected_min_height = float(rospy.get_param('~projected_min_height', 120.0))
        self.projected_padding = float(rospy.get_param('~projected_padding', 18.0))
        self.debug_projection = bool(rospy.get_param('~debug_projection', False))
        self.bin_length = float(rospy.get_param('~bin_length', 0.34))
        self.bin_width = float(rospy.get_param('~bin_width', 0.26))
        self.bin_height = float(rospy.get_param('~bin_height', 0.44))
        self.bin_center_z = float(rospy.get_param('~bin_center_z', self.bin_height * 0.5))
        self.min_marker_area = float(rospy.get_param('~min_marker_area', 35.0))
        self.expand_x = float(rospy.get_param('~expand_x', 2.8))
        self.expand_y = float(rospy.get_param('~expand_y', 2.4))
        self.hue_low = int(rospy.get_param('~hue_low', 35))
        self.hue_high = int(rospy.get_param('~hue_high', 90))
        self.saturation_low = int(rospy.get_param('~saturation_low', 55))
        self.value_low = int(rospy.get_param('~value_low', 45))
        image_topic = rospy.get_param('~image_topic', '/kinect2/hd/image_color_rect')
        camera_info_topic = rospy.get_param('~camera_info_topic', image_topic.replace('image_color_rect', 'camera_info'))
        output_topic = rospy.get_param('~output_topic', '/yolo_world/trash_bin_detections')
        self.camera_info = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.model_state_proxy = None
        self.publisher = rospy.Publisher(output_topic, DetectionArray, queue_size=2)
        self.info_subscriber = rospy.Subscriber(camera_info_topic, CameraInfo,
                                                self.camera_info_callback, queue_size=1)
        self.subscriber = rospy.Subscriber(image_topic, Image, self.image_callback,
                                           queue_size=1, buff_size=2 ** 24)
        rospy.loginfo('sim_trash_bin_detector: image=%s camera_info=%s output=%s class=%s model=%s ref=%s tf_frame=%s',
                      image_topic, camera_info_topic, output_topic,
                      self.class_name, self.model_name,
                      self.model_reference_frame, self.model_tf_frame)

    def camera_info_callback(self, message):
        self.camera_info = message
        if not self.camera_frame:
            self.camera_frame = message.header.frame_id

    def image_callback(self, message):
        result = DetectionArray()
        result.header = message.header
        result.backend = 'gazebo_projected_trash_bin'
        result.image_width = message.width
        result.image_height = message.height

        projected = self.project_model_bbox(message)
        if projected is not None:
            self.append_detection(result, projected, self.confidence)
        else:
            result.backend = 'gazebo_projected_trash_bin+green_marker_fallback'
            self.append_green_marker_fallback(result, message)

        if result.objects or self.publish_empty:
            self.publisher.publish(result)

    def append_detection(self, result, bbox, score):
        x, y, w, h = bbox
        detection = DetectionResult()
        detection.class_name = self.class_name
        detection.confidence = float(score)
        detection.x = float(x)
        detection.y = float(y)
        detection.width = float(w)
        detection.height = float(h)
        detection.has_bbox = True
        result.objects.append(detection)

    def model_state(self):
        if self.model_state_proxy is None:
            rospy.wait_for_service('/gazebo/get_model_state', timeout=0.2)
            self.model_state_proxy = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        return self.model_state_proxy(self.model_name, self.model_reference_frame)

    def project_model_bbox(self, image_message):
        if self.camera_info is None:
            return None
        try:
            state = self.model_state()
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn_throttle(5.0, 'sim_trash_bin_detector model state failed: %s', error)
            return None
        if not state.success:
            rospy.logwarn_throttle(5.0, 'sim_trash_bin_detector model %s unavailable: %s',
                                   self.model_name, state.status_message)
            return None
        camera_frame = self.camera_frame or self.camera_info.header.frame_id or image_message.header.frame_id
        points = self.bin_corners(state.pose.position)
        pixels = []
        for point in points:
            pixel = self.project_point(point, camera_frame, image_message.header.stamp)
            if pixel is not None:
                pixels.append(pixel)
        if not pixels:
            if self.debug_projection:
                rospy.logwarn_throttle(2.0, 'sim_trash_bin_detector projection produced no visible pixels; ref=%s tf_frame=%s camera=%s center=(%.3f, %.3f, %.3f)',
                                       self.model_reference_frame, self.model_tf_frame, camera_frame,
                                       state.pose.position.x, state.pose.position.y, state.pose.position.z)
            return None
        xs = [pixel[0] for pixel in pixels]
        ys = [pixel[1] for pixel in pixels]
        left = min(xs) - self.projected_padding
        right = max(xs) + self.projected_padding
        top = min(ys) - self.projected_padding
        bottom = max(ys) + self.projected_padding
        cx = 0.5 * (left + right)
        cy = 0.5 * (top + bottom)
        width = max(right - left, self.projected_min_width)
        height = max(bottom - top, self.projected_min_height)
        bbox = self.clip_bbox(cx - 0.5 * width, cy - 0.5 * height,
                              width, height, image_message.width, image_message.height)
        if self.debug_projection:
            rospy.loginfo_throttle(2.0, 'sim_trash_bin_detector bbox=%s raw=(%.1f, %.1f)-(%.1f, %.1f) center=(%.3f, %.3f, %.3f) camera=%s',
                                   bbox, left, top, right, bottom,
                                   state.pose.position.x, state.pose.position.y, state.pose.position.z, camera_frame)
        return bbox

    def bin_corners(self, center):
        half_l = 0.5 * self.bin_length
        half_w = 0.5 * self.bin_width
        z_values = [0.03, self.bin_height]
        corners = []
        for dx in (-half_l, half_l):
            for dy in (-half_w, half_w):
                for z in z_values:
                    point = PointStamped()
                    point.header.frame_id = self.model_tf_frame
                    point.header.stamp = rospy.Time(0)
                    point.point.x = center.x + dx
                    point.point.y = center.y + dy
                    point.point.z = center.z + z
                    corners.append(point)
        center_point = PointStamped()
        center_point.header.frame_id = self.model_tf_frame
        center_point.header.stamp = rospy.Time(0)
        center_point.point.x = center.x
        center_point.point.y = center.y
        center_point.point.z = center.z + self.bin_center_z
        corners.append(center_point)
        return corners

    def project_point(self, point, camera_frame, stamp):
        try:
            transformed = self.tf_buffer.transform(point, camera_frame,
                                                   rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as error:
            rospy.logwarn_throttle(5.0, 'sim_trash_bin_detector TF failed: %s', error)
            return None
        x = transformed.point.x
        y = transformed.point.y
        z = transformed.point.z
        if z <= 0.05:
            return None
        fx = self.camera_info.K[0]
        fy = self.camera_info.K[4]
        cx = self.camera_info.K[2]
        cy = self.camera_info.K[5]
        if fx <= 0.0 or fy <= 0.0:
            return None
        u = fx * x / z + cx
        v = fy * y / z + cy
        if not (math.isfinite(u) and math.isfinite(v)):
            return None
        return (u, v)

    def clip_bbox(self, left, top, width, height, image_width, image_height):
        right = min(float(image_width), left + width)
        bottom = min(float(image_height), top + height)
        left = max(0.0, left)
        top = max(0.0, top)
        if right - left < 4.0 or bottom - top < 4.0:
            return None
        return (left, top, right - left, bottom - top)

    def append_green_marker_fallback(self, result, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as error:
            rospy.logwarn_throttle(5.0, 'sim_trash_bin_detector cv_bridge failed: %s', error)
            return
        bbox = self.find_marker_bbox(image)
        if bbox is not None:
            self.append_detection(result, bbox, min(0.99, self.confidence + 0.03))

    def find_marker_bbox(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        lower = np.array([self.hue_low, self.saturation_low, self.value_low], dtype=np.uint8)
        upper = np.array([self.hue_high, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_SIMPLE)
        height, width = bgr_image.shape[:2]
        best = None
        best_score = 0.0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_marker_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            cx = x + 0.5 * w
            cy = y + 0.5 * h
            expanded_w = max(float(w) * self.expand_x, 8.0)
            expanded_h = max(float(h) * self.expand_y, 8.0)
            bbox = self.clip_bbox(cx - 0.5 * expanded_w, cy - 0.5 * expanded_h,
                                  expanded_w, expanded_h, width, height)
            if bbox is None:
                continue
            score = area * bbox[2] * bbox[3]
            if score > best_score:
                best_score = score
                best = bbox
        return best


def main():
    rospy.init_node('sim_trash_bin_detector')
    SimTrashBinDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
