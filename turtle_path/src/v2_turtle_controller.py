#!/usr/bin/env python
import rospy
import tf2_ros
import tf
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose

def controller(waypoint):
    """
    Controls the robot to move to the given waypoint using a PID controller.
    The robot will follow a rectangular path defined by the waypoints.
    """
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)

    # Define the PID controller parameters
    Kp = np.diag([2.0, 0.8])  # Adjust these values based on experimentation
    Kd = np.diag([-0.5, 0.5])
    Ki = np.diag([-0.1, 0.1])

    # Initialize previous variables for PID control
    prev_time = rospy.get_time()
    previous_error = np.zeros((2, 1), dtype=float)
    integ = np.zeros((2, 1), dtype=float)


    r = rospy.Rate(10)  # 10Hz control rate

    while not rospy.is_shutdown():
        try:
            # Get current transform between odom and base_footprint
            trans_odom_to_base_link = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time(), rospy.Duration(5))

            # Get the current robot orientation
            (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion([
                trans_odom_to_base_link.transform.rotation.x,
                trans_odom_to_base_link.transform.rotation.y,
                trans_odom_to_base_link.transform.rotation.z,
                trans_odom_to_base_link.transform.rotation.w])

            # Transform the waypoint into the base_link frame
            waypoint_trans = PoseStamped()
            waypoint_trans.pose.position.x = waypoint[0]
            waypoint_trans.pose.position.y = waypoint[1]
            waypoint_trans.pose.position.z = 0
            quat = quaternion_from_euler(0, 0, waypoint[2])
            waypoint_trans.pose.orientation.x = quat[0]
            waypoint_trans.pose.orientation.y = quat[1]
            waypoint_trans.pose.orientation.z = quat[2]
            waypoint_trans.pose.orientation.w = quat[3]

            # Transform the waypoint to the base_link frame
            waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link)
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([
                waypoint_in_base_link.pose.orientation.x,
                waypoint_in_base_link.pose.orientation.y,
                waypoint_in_base_link.pose.orientation.z,
                waypoint_in_base_link.pose.orientation.w])

            # Compute the error between current position and target waypoint
            x_error = waypoint_in_base_link.pose.position.x
            y_error = waypoint_in_base_link.pose.position.y
            yaw_error = yaw - baselink_yaw

            # Define the error vector
            error = np.array([[x_error], [yaw_error]])

            # Compute PID terms
            curr_time = rospy.get_time()
            dt = curr_time - prev_time

            # Proportional term
            proportional = np.dot(Kp, error).squeeze()

            # Integral term
            integ += error * dt
            integral = np.dot(Ki, integ).squeeze()

            # Derivative term
            error_deriv = (error - previous_error) / dt
            derivative = np.dot(Kd, error_deriv).squeeze()

            # Compute control commands
            msg = Twist()
            msg.linear.x = proportional[0] + derivative[0] + integral[0]
            msg.angular.z = proportional[1] + derivative[1] + integral[1]

            # Publish the control command
            pub.publish(msg)

            # Update previous error and time for next iteration
            previous_error = error
            prev_time = curr_time

            # Check stopping condition (small errors)
            if np.abs(x_error) <= 0.05 and np.abs(y_error) <= 0.05 and np.abs(yaw_error) <= 0.05:
                break

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF Error: %s", e)
            continue

        r.sleep()

def move_in_rectangle():
    """
    Moves the Turtlebot in a rectangular path defined by the waypoints.
    """
    # Define waypoints for the rectangle (adjust as necessary)
    waypoints = [
        [0, 0, 0],   # (x1, y1)
        [2, 0, 0],   # (x2, y1)
        [2, 0, np.pi/2],
        [2, 1, 0],   # (x2, y2)
        [2, 1, np.pi/2],   # (x2, y2)
        [0, 1, 0],
        [0, 1, np.pi/2],   # (x1, y2)
    ]

    for waypoint in waypoints:
        rospy.loginfo("Moving to waypoint: %s", waypoint)
        controller(waypoint)

if __name__ == '__main__':
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Move the robot directly in a rectangle when the script is executed
    move_in_rectangle()

    # Keep the node running until it is manually terminated
    rospy.spin()
