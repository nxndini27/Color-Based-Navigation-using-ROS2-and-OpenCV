#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge

import cv2
import numpy as np


class ColorFollower(Node):

    def __init__(self):

        super().__init__('color_follower')

        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Lidar subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # OpenCV bridge
        self.bridge = CvBridge()

        # Closest front obstacle distance
        self.front_distance = 999.0

        # Object detected flag
        self.object_detected = False

        # Controller gain
        self.kp = 0.001

        self.get_logger().info("Advanced Color Follower Started!")

    # ---------------- LIDAR CALLBACK ----------------

    def scan_callback(self, msg):

        # Front lidar ranges
        front_ranges = msg.ranges[0:20] + msg.ranges[-20:]

        valid_ranges = [
            r for r in front_ranges
            if not np.isinf(r)
        ]

        if len(valid_ranges) > 0:
            self.front_distance = min(valid_ranges)

    # ---------------- CAMERA CALLBACK ----------------

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Green HSV range
        lower_green = np.array([40, 70, 70])
        upper_green = np.array([80, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Noise removal
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        twist = Twist()

        self.object_detected = False

        # ---------------- OBJECT DETECTED ----------------

        if len(contours) > 0:

            largest = max(contours, key=cv2.contourArea)

            area = cv2.contourArea(largest)

            if area > 500:

                self.object_detected = True

                # Better center calculation using contour moments
                M = cv2.moments(largest)

                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    center_y = int(M["m01"] / M["m00"])
                else:
                    center_x = 0
                    center_y = 0

                image_center_x = frame.shape[1] // 2

                error = center_x - image_center_x

                # Draw contour
                cv2.drawContours(
                    frame,
                    [largest],
                    -1,
                    (0, 255, 0),
                    3
                )

                # Draw center
                cv2.circle(
                    frame,
                    (center_x, center_y),
                    5,
                    (0, 0, 255),
                    -1
                )

                # Draw image center line
                cv2.line(
                    frame,
                    (image_center_x, 0),
                    (image_center_x, frame.shape[0]),
                    (255, 0, 0),
                    2
                )

                # Show error
                cv2.putText(
                    frame,
                    f"Error: {error}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (255, 0, 0),
                    2
                )

                # Show lidar distance
                cv2.putText(
                    frame,
                    f"Distance: {self.front_distance:.2f}",
                    (20, 80),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    2
                )

                # ---------- OBJECT FOLLOWING ----------

                # Stop only if object is close AND centered
                if self.front_distance < 0.8 and abs(error) < 40:

                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                    self.get_logger().info("Reached Object!")

                else:

                    # P-controller steering
                    twist.angular.z = -self.kp * error

                    # Move forward only if mostly aligned
                    if abs(error) < 120:
                        twist.linear.x = 0.10
                    else:
                        twist.linear.x = 0.0

        # ---------------- SEARCH MODE ----------------

        else:

            twist.angular.z = 0.2
            twist.linear.x = 0.0

        # Publish motion
        self.cmd_pub.publish(twist)

        # Show camera
        cv2.imshow("Camera View", frame)

        # Show mask
        cv2.imshow("Green Mask", mask)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = ColorFollower()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
