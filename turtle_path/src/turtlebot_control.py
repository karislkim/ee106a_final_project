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

#Define the method which contains the main functionality of the node.
def controller(waypoint):
  """
  Controls a turtlebot whose position is denoted by turtlebot_frame,
  to go to a position denoted by target_frame
  Inputs:
  - turtlebot_frame: the tf frame of the AR tag on your turtlebot
  - goal_frame: the tf frame of the target AR tag
  """

  # Create a publisher and a tf buffer, which is primed with a tf listener
  pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  ## TODO: what topic should we publish to? how?
  tfBuffer = tf2_ros.Buffer()
  tfListener = tf2_ros.TransformListener(tfBuffer)

  # Create a timer object that will sleep long enough to result in
  # a 10Hz publishing rate
  r = rospy.Rate(10) # 10hz
  # you can also use the rate to calculate your dt, but you don't have to

  # All in the form [x, y]
  # NOTE: The Turtlebot typically does not need an integral term so we set it to 0 to make this a PD controller
  Kp = np.diag([2, 0.8]) # TODO: You may need to tune these values for your turtlebot
  Kd = np.diag([-0.5, 0.5]) # TODO: You may need to tune these values for your turtlebot
  Ki = np.diag([0, 0])

  prev_time = rospy.get_time() # TODO: initialize your time, what rospy function would be helpful here?
  integ = np.zeros((2, 1), dtype=float) # TODO: initialize an empty np array -- make sure to keep your sizes consistent
  derivative = np.zeros((2, 1), dtype=float) # TODO: initialize an empty np array 
  previous_error = np.zeros((2, 1), dtype=float) # TODO: initialize an empty np array 

  # Loop until the node is killed with Ctrl-C
  while not rospy.is_shutdown():
    try:
      #                                              target_frame, source_frame, current_time_in_ros, how long to wait for transform lookup
      trans_odom_to_base_link = tfBuffer.lookup_transform("base_footprint", "odom", rospy.Time(), rospy.Duration(5)) # TODO: create a transform between odom to base link

      (roll, pitch, baselink_yaw) = tf.transformations.euler_from_quaternion(
        [trans_odom_to_base_link.transform.rotation.x, trans_odom_to_base_link.transform.rotation.y,
            trans_odom_to_base_link.transform.rotation.z, trans_odom_to_base_link.transform.rotation.w])


      waypoint_trans = PoseStamped() # TODO: initialize a PoseStamped
      waypoint_trans.pose.position.x = waypoint[0] # TODO: what value would you use here?
      waypoint_trans.pose.position.y = waypoint[1] # TODO: what value would you use here?
      waypoint_trans.pose.position.z = 0           # TODO: what value would you use here?  # Assuming the waypoint is on the ground

      quat = quaternion_from_euler(0, 0, waypoint[2]) # TODO: what would be the inputs to this function (there are 3)
      waypoint_trans.pose.orientation.x = quat[0] # TODO: what value would you use here?
      waypoint_trans.pose.orientation.y = quat[1] # TODO: what value would you use here?
      waypoint_trans.pose.orientation.z = quat[2] # TODO: what value would you use here?
      waypoint_trans.pose.orientation.w = quat[3] # TODO: what value would you use here?

      # Use the transform to compute the waypoint's pose in the base_link frame
      waypoint_in_base_link = do_transform_pose(waypoint_trans, trans_odom_to_base_link) # TODO: what would be the inputs to this function (there are 2)
      (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
        [waypoint_in_base_link.pose.orientation.x, waypoint_in_base_link.pose.orientation.y,
            waypoint_in_base_link.pose.orientation.z, waypoint_in_base_link.pose.orientation.w])

      curr_time = rospy.get_time()

      # some debug output below
      print(f"Current: {trans_odom_to_base_link.transform.translation.x}, {trans_odom_to_base_link.transform.translation.y}, {baselink_yaw  }")
      print(f"Target: {waypoint}")

      # Process trans to get your state error
      # Generate a control command to send to the robot
      x_error = waypoint_in_base_link.pose.position.x
      error = np.array([[x_error], [yaw]])# TODO: what are two values that we can use for this np.array, and what are the dimensions
      print("ERROR: " + str(error))
      # proportional term
      proportional = np.dot(Kp, error).squeeze()
      
      # integral term
      dt = curr_time - prev_time # TODO: quick operation to determine dt
      integ += error # TODO: integral is summing up error over time, so what would we expect to add on to our integral term tracker here?
      integral = np.dot(Ki, integ).squeeze()

      # dervative term
      error_deriv = (error - previous_error) / dt # TODO: quick operation to determine dt
      derivative = np.dot(Kd, error_deriv).squeeze()

      msg = Twist()
      msg.linear.x = proportional[0] + derivative[0] + integral[0] 
      msg.angular.z = proportional[1] + derivative[1] + integral[1] 

      if returning: 
        print("RETURNING NOW") # this is just a debugging print statement

      control_command = msg
      print(control_command)

      previous_error = error  # TODO
      prev_time = curr_time
      pub.publish(control_command)

      if np.abs(error[0]) <= 0.03 and np.abs(error[1]) <= 0.2: #TODO: what is our stopping condition/how do we know to go to the next waypoint?
        print("Moving to next waypoint in trajectory")
        # import pdb; pdb.set_trace()
        return

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
      print("TF Error in Turtlebot Controller: " + e)
      pass
    # Use our rate object to sleep until it is time to publish again
    r.sleep()

def save_starting_position():
  global starting_pose
  try:
    # Get the transformation from base_footprint to odom
    # odom is the fixed frame, base_footprint is the robot frame
    # find the point in the world frame we want to go to and translate it to the robot frame
    # refer to code in this lab
    # this function can return the starting pose rather than setting it as a global variable
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
    print("TRANSFORMATION_MATRIX: " + str(transformation_matrix))

    starting_pose = np.dot(transformation_matrix, np.array([0, 0, 0, 1]))

    print("STARTING_POSE: " + str(starting_pose))
    
    print(f"Starting position saved: {starting_pose}")
    starting_pose = (starting_pose[0], starting_pose[1])
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    print(f"Error saving starting position: {e}")

def planning_callback(goal_msg, color_msg):
  global returning
  try:
    goal_point = (goal_msg.x, goal_msg.y)
    object_color = color_msg.data

    trajectory = plan_curved_trajectory(goal_point) # TODO: What is the tuple input to this function?

    # TODO: write a loop to loop over our waypoints and call the controller function on each waypoint
    returning = False
    for waypoint in trajectory:
      controller(waypoint)

    if object_color:
      returning = True
      print("Approached Orange Block")
      save_starting_position()
      return_trajectory = plan_curved_trajectory(starting_pose)
      for waypoint in return_trajectory:
        controller(waypoint)
    else:
      # go to green pile
      print('Approached Green Block')
      save_starting_position()
      return_trajectory = plan_curved_trajectory(starting_pose)
      for waypoint in return_trajectory:
        controller(waypoint)
  except rospy.ROSInterruptException as e:
    print("Exception thrown in planning callback: " + e)
    pass
      

# This is Python's sytax for a main() method, which is run by default
# when exectued in the shell
if __name__ == '__main__':
  # Check if the node has received a signal to shut down
  # If not, run the talker method

  #Run this program as a new node in the ROS computation graph 
  #called /turtlebot_controller.
  
  rospy.init_node('turtlebot_controller', anonymous=True)

  # rospy.Subscriber("/goal_point", Point, planning_callback) # TODO: what are we subscribing to here?

  # rospy.Subscriber("/object_color", Bool, planning_callback)

  goal_sub = message_filters.Subscriber("/goal_point", Point)
  color_sub = message_filters.Subscriber("/object_color", Bool)

  # Synchronize messages with allow_headerless=True
  ts = message_filters.ApproximateTimeSynchronizer(
      [goal_sub, color_sub],
      queue_size=10,
      slop=0.1,
      allow_headerless=True  
  )
  ts.registerCallback(planning_callback)
  
  rospy.spin()