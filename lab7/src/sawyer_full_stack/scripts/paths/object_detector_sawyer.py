#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo # For camera intrinsic parameters
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
import os
import time
import tf
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from std_msgs.msg import Header
from std_msgs.msg import Bool
import tf2_geometry_msgs
import tf2_ros

PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_gray_image = None

        self.grayscale_image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.grayscale_image_callback)
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=1)
        self.image_pub = rospy.Publisher("detected_block", Image, queue_size=1)

        rospy.spin()

    def camera_info_callback(self, msg):
        # Extract the intrinsic parameters from the CameraInfo message
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth=0.46):
        # Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def grayscale_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (mono8 format)
            self.cv_gray_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
            self.process_images()

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        _, binary_image = cv2.threshold(self.cv_gray_image, 127, 255, cv2.THRESH_BINARY)
        binary_image = cv2.bitwise_not(binary_image)
        y_coords, x_coords = np.nonzero(binary_image) 

        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
             print("No points detected. Is your color filter wrong?")
             return

        # Find contours after mask is applied to image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            M = cv2.moments(cnt)
            area = cv2.contourArea(cnt)
            if 1300 <= area <= 8000:  # Filter by size
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy, area))

        # TODO: Determine closest object
        closest_object = (centroids[0][0], centroids[0][1])

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(centroids[0][0], centroids[0][1])
            camera_link_x, camera_link_y, camera_link_z = camera_x, camera_y, camera_z
            
            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                self.tf_listener.waitForTransform("/base", "/right_hand_camera", rospy.Time(), rospy.Duration(10.0))
                point_odom = self.tf_listener.transformPoint("/base", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/right_hand_camera"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

                if X_odom < 0.001 and X_odom > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
                    # Publish the transformed point
                    self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))

                    # Overlay block points on color image for visualization
                    block_img = self.cv_gray_image.copy()
                    block_img = cv2.cvtColor(cup_img, cv2.COLOR_GRAY2BGR)

                    block_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                    cv2.circle(block_img, (closest_object[0], closest_object[1]), 5, [0, 255, 0], -1)  # Draw green circle at center
                    cv2.circle(block_img, (0,0), 5, [255, 0, 0], -1)
                    
                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(block_img, "bgr8")
     
                    self.image_pub.publish(ros_image)
                    
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + str(e))
                return

if __name__ == '__main__':
    ObjectDetector()
