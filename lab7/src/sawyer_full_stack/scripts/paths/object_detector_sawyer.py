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
        self.cv_depth_image = None

        self.grayscale_image_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/image_raw", Image, self.grayscale_image_callback)
        # self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/io/internal_camera/right_hand_camera/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=1)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=1)
        self.color_pub = rospy.Publisher("object_color", Bool, queue_size=1)
        # self.tf_buffer = tf2_ros.Buffer()
        # self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.spin()

    def camera_info_callback(self, msg):
        # TODO: Extract the intrinsic parameters from the CameraInfo message (look this message type up online)
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth=0.46):
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
            #if self.cv_depth_image is not None:
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
        # rospy.sleep(10)
        # Convert the color image to HSV color space
        #hsv = cv2.cvtColor(self.cv_gray_image, cv2.COLOR_BGR2HSV)
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
        _, binary_image = cv2.threshold(self.cv_gray_image, 127, 255, cv2.THRESH_BINARY)
        binary_image = cv2.bitwise_not(binary_image)
        y_coords, x_coords = np.nonzero(binary_image) # dont forget to change the variable of mask to to combined mask

        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
             print("No points detected. Is your color filter wrong?")

             return

        # TODO: Find contours after mask is applied to image
        contours, _ = cv2.findContours(binary_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        valid_objects = []
        for cnt in contours:
            M = cv2.moments(cnt)
            area = cv2.contourArea(cnt)
            if 1300 <= area <= 8000:  # Filter by size
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centroids.append((cx, cy))
                    valid_objects.append((cx, cy, area))  # Save centroid and area for valid objects

        print("VALID_oBJecTS: " + str(valid_objects))

        # TODO: Determine closest object
        closest_object = None
        min_distance = float('inf')
        # for cx, cy in centroids:
        #     # distance = self.cv_depth_image[cy, cx]
        #     distance = (cx**2 + cy**2)**(0.5)
        #     if distance < min_distance:
        #         min_distance = distance
        #         closest_object = (cx, cy) # center of closest object
        closest_object = (centroids[0][0], centroids[0][1])
   
        # Calculate the center of the detected region by
        center_x = int(np.mean(x_coords))
        center_y = int(np.mean(y_coords))
        print("CENTER X, Y: " + str(center_x) + ", " + str(center_y))

        # # Fetch the depth value at the center
        # depth = self.cv_depth_image[closest_object[1], closest_object[0]]
        # is_green = None


        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(centroids[0][0], centroids[0][1])
            
            # camera_link_x, camera_link_y, camera_link_z = camera_y, camera_x, camera_z
            camera_link_x, camera_link_y, camera_link_z = camera_x, camera_y, camera_z
            # camera_link_x, camera_link_y, camera_link_z = -camera_z, -camera_y, camera_x
           
            # print("CAMERA LINK VALS: " + str(camera_link_x) + ", " + str(camera_link_y) + ", " + str(camera_link_z))

            # Convert the (X, Y, Z) coordinates from camera frame to odom frame
            try:
                # self.tf_listener.waitForTransform("/right_hand", "/right_hand_camera", rospy.Time(), rospy.Duration(10.0))
               

                # point_gripper = self.tf_listener.transformPoint("/right_hand", PointStamped(header=Header(stamp=rospy.Time(), frame_id="/right_hand_camera"), point=Point(camera_link_x, camera_link_y, camera_link_z)))
                # X_gripper, Y_gripper, Z_gripper = point_gripper.point.x, point_gripper.point.y, point_gripper.point.z

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

                    if np.abs(0.55 - X_odom) <= 0.15 and np.abs(0.20 - Y_odom) <= 0.05:
                        is_orange = True
                    else:
                        is_orange = False

                    print("Publishing object color (orange?): ", is_orange)
                    self.color_pub.publish(is_orange)

                    # Overlay cup points on color image for visualization
                    cup_img = self.cv_gray_image.copy()
                    cup_img = cv2.cvtColor(cup_img, cv2.COLOR_GRAY2BGR)

                    cup_img[y_coords, x_coords] = [0, 0, 255]  # Highlight cup points in red
                    cv2.circle(cup_img, (closest_object[0], closest_object[1]), 5, [0, 255, 0], -1)  # Draw blue circle at center
                    cv2.circle(cup_img, (0,0), 5, [255, 0, 0], -1)
                    # Convert to ROS Image message and publish

                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
     
                    self.image_pub.publish(ros_image)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + str(e))
                return

if __name__ == '__main__':
    ObjectDetector()