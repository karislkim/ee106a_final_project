#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from intera_interface import Limb
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import message_filters
import rospkg
import roslaunch
import math
from scipy.spatial.transform import Rotation as R

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION



def position_camera(j0, j1, j2, j3, j4):
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on Sawyer's arm. The increasing and descreasing
    are represented by number key and letter key next to the number.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                        "Exiting."), "ERROR")
        return

    # print("Initializing node... ")
    # rospy.init_node("sdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    rospy.loginfo("Enabling robot...")
    rs.enable()


    #map joint angles to dictionary
    joint_angles_dict = {'right_j0': j0, 'right_j1' : j1, 
        'right_j2' : j2, 'right_j3' : j3, 'right_j4' : j4, 
        'right_j5' : -0.4, 'right_j6' : 1.7}

    limb = intera_interface.Limb('right')
    print(joint_angles_dict)

    # while True:
    limb.move_to_joint_positions(joint_angles_dict)
        # done = input('is it done? (y/n): ')
        # if done == "y":
        #     break



def main(goal_msg, color_msg):
    print("GOAL_POINT: " + str(goal_msg))
    x = goal_msg.x
    y = goal_msg.y

    is_orange = color_msg.data
    print("IS_ORANGE: " + str(is_orange))

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    
    # Construct the request
    approach1_request = GetPositionIKRequest()
    approach1_request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    approach1_request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    approach1_request.ik_request.pose_stamped.header.frame_id = "base"

    yaw_to_align = math.atan2(y, x)
    rpy = [math.radians(-90), math.radians(90), yaw_to_align]
    quaternion = R.from_euler('xyz', rpy).as_quat()
    print("QUAT: " + str(quaternion))
    # z = -0.15
    z = 0.05
    # Set the desired orientation for the end effector HERE
    approach1_request.ik_request.pose_stamped.pose.position.x = x
    approach1_request.ik_request.pose_stamped.pose.position.y = y
    approach1_request.ik_request.pose_stamped.pose.position.z = z   
    approach1_request.ik_request.pose_stamped.pose.orientation.x = 0
    approach1_request.ik_request.pose_stamped.pose.orientation.y = 1
    approach1_request.ik_request.pose_stamped.pose.orientation.z = 0
    approach1_request.ik_request.pose_stamped.pose.orientation.w = 0

    print("REQUEST: " + str(approach1_request))

    # # Construct the 2nd request
    # request2 = GetPositionIKRequest()
    # request2.ik_request.group_name = "right_arm"

    # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    # link = "right_gripper_tip"

    # request2.ik_request.ik_link_name = link
    # # request.ik_request.attempts = 20
    # request2.ik_request.pose_stamped.header.frame_id = "base"
    
    # # Set the desired orientation for the end effector HERE
    # request2.ik_request.pose_stamped.pose.position.x = x
    # request2.ik_request.pose_stamped.pose.position.y = y
    # request2.ik_request.pose_stamped.pose.position.z = -0.01     
    # request2.ik_request.pose_stamped.pose.orientation.x = 0.0
    # request2.ik_request.pose_stamped.pose.orientation.y = 1.0
    # request2.ik_request.pose_stamped.pose.orientation.z = 0.0
    # request2.ik_request.pose_stamped.pose.orientation.w = 0.0

    # # Construct the 3rd request
    # request3 = GetPositionIKRequest()
    # request3.ik_request.group_name = "right_arm"

    # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    # link = "right_gripper_tip"

    # request3.ik_request.ik_link_name = link
    # # request.ik_request.attempts = 20
    # request3.ik_request.pose_stamped.header.frame_id = "base"
    
    # # Set the desired orientation for the end effector HERE
    # request3.ik_request.pose_stamped.pose.position.x = 0.466
    # request3.ik_request.pose_stamped.pose.position.y = -0.107
    # request3.ik_request.pose_stamped.pose.position.z = 0.019  
    # request3.ik_request.pose_stamped.pose.orientation.x = 0.0
    # request3.ik_request.pose_stamped.pose.orientation.y = 1.0
    # request3.ik_request.pose_stamped.pose.orientation.z = 0.0
    # request3.ik_request.pose_stamped.pose.orientation.w = 0.0
        
    # # Construct the 4th request
    # request4 = GetPositionIKRequest()
    # request4.ik_request.group_name = "right_arm"

    # # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    # link = "right_gripper_tip"

    # request4.ik_request.ik_link_name = link
    # # request.ik_request.attempts = 20
    # request4.ik_request.pose_stamped.header.frame_id = "base"
    
    # # Set the desired orientation for the end effector HERE
    # request4.ik_request.pose_stamped.pose.position.x = 0.436
    # request4.ik_request.pose_stamped.pose.position.y = 0.240
    # request4.ik_request.pose_stamped.pose.position.z = 0.043    
    # request4.ik_request.pose_stamped.pose.orientation.x = 0.0
    # request4.ik_request.pose_stamped.pose.orientation.y = 1.0
    # request4.ik_request.pose_stamped.pose.orientation.z = 0.0
    # request4.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:
        # Send the request to the service
        response = compute_ik(approach1_request)
        
        # Print the response HERE
        print(response)
        
        group = MoveGroupCommander("right_arm")
        
        # Setting position and orientation target
        group.set_pose_target(approach1_request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        #group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK
        plan = group.plan()
        # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        
        # Execute IK if safe
        # if user_input == 'y':
        print("approach1 is true")
        print('Opening...')
        right_gripper.open()
        rospy.sleep(1.0)
        group.execute(plan[1])
        limb = Limb("right")
        joint_angles = limb.joint_angles()
        position_camera(joint_angles['right_j0'], joint_angles['right_j1'], joint_angles['right_j2'], joint_angles['right_j3'], joint_angles['right_j4'])
            
        # print("approach1 is false")
        # group.execute(plan[1])
        # print('Closing...')
        # right_gripper.close()
        # rospy.sleep(1.0)
        # approach1 = True


        # response2 = compute_ik(request2)
        # print(response2)
        # group2 = MoveGroupCommander("right_arm")
        # group2.set_pose_target(request2.ik_request.pose_stamped)
        # plan2 = group2.plan()
        # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        # # Execute IK if safe
        # if user_input == 'y':
        #     group2.execute(plan2[1])
        #     # Close the right gripper
        #     print('Closing...')
        #     right_gripper.close()
        #     rospy.sleep(1.0)

        # if is_orange:
        #     response3 = compute_ik(request3)
        #     print(response3)
        #     group3 = MoveGroupCommander("right_arm")
        #     group3.set_pose_target(request3.ik_request.pose_stamped)
        #     plan3 = group3.plan()
        #     user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        #     # Execute IK if safe
        #     if user_input == 'y':
        #         group3.execute(plan3[1])
        #         print('Opening...')
        #         right_gripper.open()
        #         rospy.sleep(1.0)
        #         print('Done!')
        # else:
        #     print("dropping block off")
        #     response4 = compute_ik(request4)
        #     print(response4)
        #     group4 = MoveGroupCommander("right_arm")
        #     group4.set_pose_target(request4.ik_request.pose_stamped)
        #     plan4 = group4.plan()
        #     user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        #     # Execute IK if safe
        #     if user_input == 'y':
        #         group4.execute(plan4[1])
        #         print('Opening...')
        #         right_gripper.open()
        #         rospy.sleep(1.0)
        #         print('Done!')

            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        pass

# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('service_query')
    # goal_point_sub = rospy.Subscriber("goal_point", Point, main)
    goal_sub = message_filters.Subscriber("/goal_point", Point)
    color_sub = message_filters.Subscriber("/object_color", Bool)

    # Synchronize messages with allow_headerless=True
    ts = message_filters.ApproximateTimeSynchronizer(
        [goal_sub, color_sub],
        queue_size=10,
        slop=0.1,
        allow_headerless=True  
    )
    ts.registerCallback(main)
    rospy.spin()
