#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class SphereSearcher(Node):
    def __init__(self):
        super().__init__('sphere_searcher')
        self.bridge = CvBridge()
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10)
        self.is_searching = True
        self.found_sphere = False
        self.create_timer(0.1, self.search_timer_callback)
        self.get_logger().info('Sphere Searcher Node Started')

    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red, upper_red)
            
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                area = cv2.contourArea(largest_contour)
                if area > 100:
                    self.found_sphere = True
                    self.is_searching = False
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        error = cx - cv_image.shape[1]/2
                        self.move_robot(error)
        except Exception as e:
            self.get_logger().error(f'Error in camera callback: {str(e)}')

    def move_robot(self, error):
        twist = Twist()
        twist.angular.z = -float(error) * 0.005
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Tracking sphere. Error: {error}')

    def search_timer_callback(self):
        if self.is_searching and not self.found_sphere:
            twist = Twist()
            twist.angular.z = 0.5
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Searching for sphere...')

def main():
    rclpy.init()
    node = SphereSearcher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()