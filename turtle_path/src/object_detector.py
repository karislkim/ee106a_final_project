#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, RegionOfInterest
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

        self.color_image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.color_image_callback)
        self.depth_image_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_image_callback)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.camera_info_sub = rospy.Subscriber("/camera/color/camera_info", CameraInfo, self.camera_info_callback)

        self.tf_listener = tf.TransformListener()  # Create a TransformListener object

        self.point_pub = rospy.Publisher("goal_point", Point, queue_size=1)
        self.image_pub = rospy.Publisher('detected_cup', Image, queue_size=1)
        self.color_pub = rospy.Publisher("object_color", Bool, queue_size=1)


        rospy.spin()

    def camera_info_callback(self, msg):
        #Extract the intrinsic parameters from the CameraInfo message 
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]

    def pixel_to_point(self, u, v, depth):
        # Use the camera intrinsics to convert pixel coordinates to real-world coordinates
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z

    def color_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (BGR8 format)
            self.cv_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.cv_depth_image is not None:
                self.process_images()

        except Exception as e:
            print("Error:", e)

    def depth_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image (16UC1 format)
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")

        except Exception as e:
            print("Error:", e)

    def process_images(self):
        
        height, width = self.cv_color_image.shape[:2]
        cutoff_line_y = round(height*0.6)
        mask = np.zeros((height, width), dtype=np.uint8)
        mask[cutoff_line_y:, :] = 255  # Get rid of background points above the line
        
        # Apply the mask to the color and depth images
        masked_color = cv2.bitwise_and(self.cv_color_image, self.cv_color_image, mask=mask)
        masked_depth = cv2.bitwise_and(self.cv_depth_image, self.cv_depth_image, mask=mask)


        # Set blue and orange hsv values
        hsv = cv2.cvtColor(masked_color, cv2.COLOR_BGR2HSV)

        blue_lower_hsv = np.array([96, 137, 80])
        blue_upper_hsv = np.array([179, 255, 255])

        orange_lower_hsv = np.array([0, 142, 162])
        orange_upper_hsv = np.array([179, 255, 255])

        # Create blue and orange masks, then combine
        
        blue_mask = cv2.inRange(hsv, blue_lower_hsv, blue_upper_hsv)
        orange_mask = cv2.inRange(hsv, orange_lower_hsv, orange_upper_hsv)
        combined_mask = cv2.bitwise_or(orange_mask,blue_mask)

        y_coords, x_coords = np.nonzero(combined_mask) 
        
        # If there are no detected points, exit
        if len(x_coords) == 0 or len(y_coords) == 0:
             print("No points detected. Is your color filter wrong?")
             return

        # TODO: Find contours after mask is applied to image
        contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        centroids = []
        for cnt in contours:
            

            M = cv2.moments(cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                # ADDED THIS AT 12/19 9:12PM
                # depth = self.cv_depth_image[cy, cx]
                # if depth == 0 or np.isnan(depth):
                #     print(f"Skipping point at ({cx}, {cy}) due to invalid depth")
                #     continue

                # ADDED THIS AT 12/19 9:12PM
                centroids.append((cx, cy))

        print("CENTROIDS: " + str(centroids))

        # Determine closest object based on centroids of their contours
        closest_object = None
        min_distance = float('inf')
        for cx, cy in centroids:
            distance = self.cv_depth_image[cy, cx]
            if distance < min_distance:
                min_distance = distance
                closest_object = (cx, cy) # center of closest object

        # Get the depth value at the center
        depth = self.cv_depth_image[closest_object[1], closest_object[0]]
        
        is_blue = None

        if self.fx and self.fy and self.cx and self.cy:
            camera_x, camera_y, camera_z = self.pixel_to_point(closest_object[0], closest_object[1], depth)
            camera_link_x, camera_link_y, camera_link_z = camera_z, -camera_x, -camera_y
            # Convert from mm to m
            camera_link_x /= 1000
            camera_link_y /= 1000
            camera_link_z /= 1000

            # determine if block is orange or blue 
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
                    cv2.circle(cup_img, (closest_object[0], closest_object[1]), 5, [0, 255, 0], -1)  # Draw green circle at center
                    
                    cv2.line(cup_img, (0, cutoff_line_y), (width, cutoff_line_y), (0, 255, 0), 2)

                    # Convert to ROS Image message and publish
                    ros_image = self.bridge.cv2_to_imgmsg(cup_img, "bgr8")
                    self.image_pub.publish(ros_image)


            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print("TF Error: " + e)
                return

if __name__ == '__main__':
    ObjectDetector()
