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

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION

test = True
def goal_callback(goal_msg):
    global processing_goal, test

    # Check if a goal is already being processed
    if test:
        rospy.loginfo("Currently processing a goal. Ignoring new goal message.")
        rospy.sleep(2)
        test = False
        return  # Ignore the new message while processing the current goal
    rospy.loginfo(f"Received goal point: {goal_msg}")
    main(goal_msg)
    test = True
    print("COMPLETE")

def position_camera(j0, j1, j2, j3, j4, j5, j6, approach1):
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
    if approach1:
        joint_angles_dict = {'right_j0': j0, 'right_j1' : j1, 
            'right_j2' : j2, 'right_j3' : j3, 'right_j4' : j4, 
            'right_j5' : -0.4, 'right_j6' : 1.7}
    else:
        joint_angles_dict = {'right_j0': 0, 'right_j1' : -1, 
            'right_j2' : 0, 'right_j3' : 1.5, 'right_j4' : 0, 
            'right_j5' : -0.5, 'right_j6' : 1.7}

    limb = intera_interface.Limb('right')

    # while True:
    limb.move_to_joint_positions(joint_angles_dict)
        # done = input('is it done? (y/n): ')
        # if done == "y":
        #     break


current_state = "INITIAL_APPROACH"

def main(goal_msg):
    global current_state, is_orange

    # is_orange = color_msg.data
    # print("IS_ORANGE: " + str(is_orange))

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')

    try:
        if current_state == "INITIAL_APPROACH":
            print("Performing initial approach...")
            # Construct the request
            approach_request = GetPositionIKRequest()
            approach_request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            approach_request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            approach_request.ik_request.pose_stamped.header.frame_id = "base"
            x = goal_msg.x
            y = goal_msg.y


            if np.abs(x - 0.62) <= 0.05 and np.abs(y - 0.34) <= 0.05:
                is_orange = True
            else:
                is_orange = False

            # Set the desired orientation for the end effector HERE
            approach_request.ik_request.pose_stamped.pose.position.x = x
            approach_request.ik_request.pose_stamped.pose.position.y = y
            approach_request.ik_request.pose_stamped.pose.position.z = 0.05   
            approach_request.ik_request.pose_stamped.pose.orientation.x = 0
            approach_request.ik_request.pose_stamped.pose.orientation.y = 1
            approach_request.ik_request.pose_stamped.pose.orientation.z = 0
            approach_request.ik_request.pose_stamped.pose.orientation.w = 0

            # print("REQUEST: " + str(approach_request))

            response = compute_ik(approach_request)
            
            group = MoveGroupCommander("right_arm")
            
            # Setting position and orientation target
            group.set_pose_target(approach_request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            # user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            # Execute IK if safe
            if user_input == 'y':
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)
                group.execute(plan[1])
                limb = Limb("right")
                joint_angles = limb.joint_angles()
                position_camera(joint_angles['right_j0'], joint_angles['right_j1'], joint_angles['right_j2'], joint_angles['right_j3'], joint_angles['right_j4'], joint_angles['right_j5'], joint_angles['right_j6'], True)
                
            current_state = "FINAL_PICKUP"
            return
        elif current_state == "FINAL_PICKUP":
            print("Performing final pickup")

            x = goal_msg.x
            y = goal_msg.y


            print("IS ORANGE?: " + str(is_orange))

            # Construct the 2nd request
            pickup_request = GetPositionIKRequest()
            pickup_request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            pickup_request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            pickup_request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            pickup_request.ik_request.pose_stamped.pose.position.x = x + 0.015
            pickup_request.ik_request.pose_stamped.pose.position.y = y
            pickup_request.ik_request.pose_stamped.pose.position.z = -0.15     
            pickup_request.ik_request.pose_stamped.pose.orientation.x = 0.0
            pickup_request.ik_request.pose_stamped.pose.orientation.y = 1.0
            pickup_request.ik_request.pose_stamped.pose.orientation.z = 0.0
            pickup_request.ik_request.pose_stamped.pose.orientation.w = 0.0


            # Construct the 3rd request
            orange_request = GetPositionIKRequest()
            orange_request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            orange_request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            orange_request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            orange_request.ik_request.pose_stamped.pose.position.x = 0.488
            orange_request.ik_request.pose_stamped.pose.position.y = -0.690
            orange_request.ik_request.pose_stamped.pose.position.z = 0.098 
            orange_request.ik_request.pose_stamped.pose.orientation.x = 0
            orange_request.ik_request.pose_stamped.pose.orientation.y = 1
            orange_request.ik_request.pose_stamped.pose.orientation.z = 0
            orange_request.ik_request.pose_stamped.pose.orientation.w = 0

            # Construct the 4th request
            blue_request = GetPositionIKRequest()
            blue_request.ik_request.group_name = "right_arm"

            # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
            link = "right_gripper_tip"

            blue_request.ik_request.ik_link_name = link
            # request.ik_request.attempts = 20
            blue_request.ik_request.pose_stamped.header.frame_id = "base"
            
            # Set the desired orientation for the end effector HERE
            blue_request.ik_request.pose_stamped.pose.position.x = 0.766
            blue_request.ik_request.pose_stamped.pose.position.y = -0.583
            blue_request.ik_request.pose_stamped.pose.position.z = 0.067    
            blue_request.ik_request.pose_stamped.pose.orientation.x = 0.0
            blue_request.ik_request.pose_stamped.pose.orientation.y = 1.0
            blue_request.ik_request.pose_stamped.pose.orientation.z = 0.0
            blue_request.ik_request.pose_stamped.pose.orientation.w = 0.0


            response2 = compute_ik(pickup_request)
            group2 = MoveGroupCommander("right_arm")
            group2.set_pose_target(pickup_request.ik_request.pose_stamped)
            plan2 = group2.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            # Execute IK if safe
            if user_input == 'y':
                group2.execute(plan2[1])
                # Close the right gripper
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)


            if is_orange:
                response3 = compute_ik(orange_request)
                group3 = MoveGroupCommander("right_arm")
                group3.set_pose_target(orange_request.ik_request.pose_stamped)
                plan3 = group3.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                # Execute IK if safe
                if user_input == 'y':
                    group3.execute(plan3[1])
                    print('Opening...')
                    right_gripper.open()
                    rospy.sleep(1.0)
                    print('Done!')
            else:
                response4 = compute_ik(blue_request)
                group4 = MoveGroupCommander("right_arm")
                group4.set_pose_target(blue_request.ik_request.pose_stamped)
                plan4 = group4.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                # Execute IK if safe
                if user_input == 'y':
                    group4.execute(plan4[1])
                    print('Opening...')
                    right_gripper.open()
                    rospy.sleep(1.0)
                    print('Done!')

            limb = Limb("right")
            joint_angles = limb.joint_angles()
            position_camera(joint_angles['right_j0'], joint_angles['right_j1'], joint_angles['right_j2'], joint_angles['right_j3'], joint_angles['right_j4'], joint_angles['right_j5'], joint_angles['right_j6'], False)
                

            current_state = "INITIAL_APPROACH"
            return
        
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return

# Python's syntax for a main() method
if __name__ == '__main__':
    rospy.init_node('service_query')
    goal_point_sub = rospy.Subscriber("goal_point", Point, goal_callback, queue_size=1)
    # goal_sub = message_filters.Subscriber("/goal_point", Point)
    # color_sub = message_filters.Subscriber("/object_color", Bool)

    # # Synchronize messages with allow_headerless=True
    # ts = message_filters.ApproximateTimeSynchronizer(
    #     [goal_sub, color_sub],
    #     queue_size=10,
    #     slop=0.1,
    #     allow_headerless=True  
    # )
    # ts.registerCallback(main)
    rospy.spin()
