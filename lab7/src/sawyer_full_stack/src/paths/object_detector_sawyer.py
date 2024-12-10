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
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from std_msgs.msg import Bool


PLOTS_DIR = os.path.join(os.getcwd(), 'plots')

class ObjectDetector:
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)

        self.bridge = CvBridge()

        self.cv_color_image = None
        self.cv_depth_image = None

        self.grayscale_image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.grayscale_image_callback)
        # self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=10)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=10)
        # self.color_pub = rospy.Publisher("object_color", Bool, queue_size=10)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # TODO: Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def grayscale_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (BGR8 format)
            # self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.cv_gray_image = self.bridge.imgmsg_to_cv2(msg, "mono8")


            # If we have both color and depth images, process them
            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert the ROS Image message to an OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        # Convert the color image to HSV color space
        hsv = cv2.cvtColor(self.cv_color_image, cv2.COLOR_BGR2HSV)
        # Run `python hsv_color_thresholder.py` and tune the bounds so you only see your cup
        # update lower_hsv and upper_hsv directly

        # Green item HSV values (represents recycling)
        # green_lower_hsv = np.array([57, 110, 39]) 
        # green_upper_hsv = np.array([89, 255, 255])

        # TODO: get hsv values for red
        # green_lower_hsv = np.array([0, 120, 120])
        # green_upper_hsv = np.array([85, 255, 255])

        # orange_lower_hsv = np.array([0, 142, 162])
        # orange_upper_hsv = np.array([179, 255, 255])

        # Threshold the image to get only green color
        # HINT: Lookup cv2.inRange()
        # green_mask = cv2.inRange(hsv, green_lower_hsv, green_upper_hsv)
        # orange_mask = cv2.inRange(hsv, orange_lower_hsv, orange_upper_hsv)

        # # TODO: Combine the masks using cv2.bitwise_or
        # combined_mask = cv2.bitwise_or(green_mask, orange_mask)


        # TODO: Get the coordinates of the closest object
        # HINT: Lookup np.nonzero()
        # y_coords, x_coords = np.nonzero(combined_mask) # dont forget to change the variable of mask to to combined mask

        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
             print("No points detected. Is your color filter wrong?")

             return

        _, binary_image = cv2.threshold(self.cv_gray_image, 127, 255, cv2.THRESH_BINARY)

        # TODO: Find contours after mask is applied to image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centroids.append((cx, cy))
        print("CENTROIDS: " + str(centroids))

        # TODO: Determine closest object
        closest_object = None
        min_distance = float('inf')
        for cx, cy in centroids:
            distance = self.cv_depth_image[cy, cx]
            if distance < min_distance:
                min_distance = distance
                closest_object = (cx, cy) # center of closest object
   
        # # Calculate the center of the detected region by 
        # center_x = int(np.mean(x_coords))
        # center_y = int(np.mean(y_coords))

        # Fetch the depth value at the center
        depth = self.cv_depth_image[closest_object[1], closest_object[0]]
        is_green = None

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(closest_object[0], closest_object[1], depth)
            camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            # determine if block is orange or green # TEST THIS ON WEDNESDAY 12/4
            center_x, center_y = closest_object
            is_orange = orange_mask[center_y, center_x] > 0
            print("IS IT ORANGE: " + str(is_orange))


            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                self.tf_listener.waitForTransform("/odom", "/camera_link", rospy.Time(), rospy.Duration(10.0))
                point_odom = self.tf_listener.transformPoint("/odom", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/camera_link"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                X_odom, Y_odom, Z_odom = point_odom.point.x, point_odom.point.y, point_odom.point.z
                print("Real-world coordinates in odom frame: (X, Y, Z) = ({:.2f}m, {:.2f}m, {:.2f}m)".format(X_odom, Y_odom, Z_odom))

                if X_odom < 0.001 and X_odom > -0.001:
                    print("Erroneous goal point, not publishing - Is the cup too close to the camera?")
                else:
                    print("Publishing goal point: ", X_odom, Y_odom, Z_odom)
                    # Publish the transformed point
                    self.point_pub.publish(Point(X_odom, Y_odom, Z_odom))

                    print("Publishing object color (orange?): ", is_orange)
                    self.color_pub.publish(is_orange)

                    # Overlay cup points on color image for visualization
                    cup_img = self.cv_color_image.copy()
                    cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                    cv2.circle(cup_img, (closest_object[0], closest_object[1]), 5, [255, 0, 0], -1)  # Draw green circle at center
                    
                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                    self.image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    ObjectDetector()