#!/usr/bin/env python
#The line above tells Linux that this file is a Python script,
#and that the OS should use the Python interpreter in /usr/bin/env
#to run it. Don't forget to use "chmod +x [filename]" to make
#this script executable.

#Import the rospy package. For an import to work, it must be specified
#in both the package manifest AND the Python file in which it is used.
import rospy
import tf2_ros
import tf
import sys
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist, Point
from tf.transformations import quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from trajectory import plan_curved_trajectory
from std_msgs.msg import Bool
import message_filters

#Global variables 
ready_to_detect = True
goal_subscriber = None
test = True

def controller(waypoint):
 # Controls a turtlebot whose position is denoted by turtlebot_frame,
 # to go to a position denoted by target_frame
   pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  r = rospy.Rate(10) # 10hz

  # All in the form [x, y]
  Kp = np.diag([2, 0.8])
  Kd = np.diag([-0.5, 0.5])
  Ki = np.diag([0, 0])

  prev_time = rospy.get_time() 
  integ = np.zeros((2, 1), dtype=float) 
  derivative = np.zeros((2, 1), dtype=float) 
  previous_error = np.zeros((2, 1), dtype=float)
 
  while not rospy.is_shutdown():
    try:
      trans_odom_to_base_link = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time(), rospy.Duration(5)) 
      (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
        [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
            trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])


      waypoint_trans = PoseStamped() 
      waypoint_trans.pose.position.x = waypoint[0] 
      waypoint_trans.pose.position.y = waypoint[1] 
      waypoint_trans.pose.position.z = 0         

      quat = quaternion_from_euler(0, 0, waypoint[2]) 
      waypoint_trans.pose.orientation.x = quat[0]
      waypoint_trans.pose.orientation.y = quat[1] 
      waypoint_trans.pose.orientation.z = quat[2] 
      waypoint_trans.pose.orientation.w = quat[3] 

      waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link) # TODO: what would be the inputs to this function (there are 2)
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
            waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])

      curr_time = rospy.get_time()
      x_error = waypoint_in_base_link.pose.position.x
      error = np.array([[x_error], [yaw]])# TODO: what are two values that we can use for this np.array, and what are the dimensions
     
    
      proportional = np.dot(Kp, error).squeeze()
      dt = curr_time - prev_time
      integ += error 
      integral = np.dot(Ki, integ).squeeze()

      error_deriv = (error - previous_error) / dt # TODO: quick operation to determine dt
      derivative = np.dot(Kd, error_deriv).squeeze()

      msg = Twist()
      msg.linear.x = proportional[0] + derivative[0] + integral[0] 
      msg.angular.z = proportional[1] + derivative[1] + integral[1] 

      if returning: 
      control_command = msg
      previous_error = error  
      prev_time = curr_time
      pub.publish(control_command)

      if np.abs(error[0]) <= 0.03 and np.abs(error[1]) <= 0.2: 
        return

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print("TF Error in Turtlebot Controller: " + e)
      pass
    r.sleep()

def save_starting_position():
  global starting_pose
  

  try:
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    start_transform = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time(), rospy.Duration(5))
    rotation_matrix = tf.transformations.quaternion_matrix([start_transform.transform.rotation.x, 
                                                            start_transform.transform.rotation.y,
                                                            start_transform.transform.rotation.z,
                                                            start_transform.transform.rotation.w])
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, :3] = rotation_matrix[:3, :3]
    transformation_matrix[:3, 3] = [start_transform.transform.translation.x,
                                    start_transform.transform.translation.y,
                                    start_transform.transform.translation.z]
    starting_pose = np.dot(transformation_matrix, np.array([0, 0, 0, 1]))
    starting_pose = (starting_pose[0], starting_pose[1])
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    print(f"Error saving starting position: {e}")


def goal_callback(goal_msg, color_msg):
  global processing_goal, test
  if test:
      rospy.loginfo("Currently processing a goal. Ignoring new goal message.")
      rospy.sleep(3)
      test = False
      return  
  rospy.loginfo(f"Received goal point: {goal_msg}")
  planning(goal_msg, color_msg)
  test = True

def planning(goal_msg, color_msg):
  global ready_to_detect, returning
  if not ready_to_detect:
    print("not ready to detect")
    return

  try:
    goal_point = (goal_msg.x, goal_msg.y)
    object_color = color_msg.data
    orange_trash_offset = (0.10, -0.20)  
    trajectory = plan_curved_trajectory(goal_point) # TODO: What is the tuple input to this function?
    returning = False
    for waypoint in trajectory:
      controller(waypoint)
    returning = True
    if object_color:
      
      print(f"_______________________________________{object_color}_________________________________________________________")
      save_starting_position()
      orange_trash_pile = (starting_pose[0] + orange_trash_offset[0],
                    starting_pose[1] + orange_trash_offset[1])
      return_trajectory = plan_curved_trajectory(orange_trash_pile)
      for index, waypoint in enumerate(return_trajectory):
        
        if index == 0:
          continue
    
        if index == 8:
          controller(return_trajectory[5])
        controller(waypoint)
    else:
      print('Approached Green Block')
      save_starting_position()
      return_trajectory = plan_curved_trajectory(starting_pose)
      for index, waypoint in enumerate(return_trajectory):
        if index == 0:
          continue
        if index == 8:
          controller(return_trajectory[5])
        controller(waypoint)
    returning = False    
  except rospy.ROSInterruptException as e:
    print("Exception thrown in planning callback: " + e)
    pass

if __name__ == '__main__':
  rospy.init_node('turtlebot_controller', anonymous=True)
  goal_sub = message_filters.Subscriber("/goal_point", Point)
  color_sub = message_filters.Subscriber("/object_color", Bool)
  ts = message_filters.ApproximateTimeSynchronizer(
      [goal_sub, color_sub],
      queue_size=1,
      slop=0.1,
      allow_headerless=True  
  )
  ts.registerCallback(goal_callback)
  
  rospy.spin()
