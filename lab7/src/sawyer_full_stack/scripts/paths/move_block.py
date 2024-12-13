#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import message_filters
import rospkg
import roslaunch

def main(goal_msg, color_msg):
    print("GOAL_POINT: " + str(goal_msg))
    x = goal_msg.x
    y = goal_msg.y

    is_orange = color_msg.data
    print("IS_ORANGE: " + str(is_orange))

    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    print("test")
    # rospy.init_node('gripper_test')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')
    # while not rospy.is_shutdown():
    input('Press [ Enter ]: ')
    
    # Construct the request
    request = GetPositionIKRequest()
    request.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request.ik_request.pose_stamped.pose.position.x = x
    request.ik_request.pose_stamped.pose.position.y = y
    request.ik_request.pose_stamped.pose.position.z = 0.15   
    request.ik_request.pose_stamped.pose.orientation.x = -0.016
    request.ik_request.pose_stamped.pose.orientation.y = 0.706
    request.ik_request.pose_stamped.pose.orientation.z = -0.020
    request.ik_request.pose_stamped.pose.orientation.w = 0.707

    print("REQUEST: " + str(request))

    # Construct the 2nd request
    request2 = GetPositionIKRequest()
    request2.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request2.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request2.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request2.ik_request.pose_stamped.pose.position.x = x
    request2.ik_request.pose_stamped.pose.position.y = y
    request2.ik_request.pose_stamped.pose.position.z = -0.16      
    request2.ik_request.pose_stamped.pose.orientation.x = 0.0
    request2.ik_request.pose_stamped.pose.orientation.y = 1.0
    request2.ik_request.pose_stamped.pose.orientation.z = 0.0
    request2.ik_request.pose_stamped.pose.orientation.w = 0.0

    # Construct the 3rd request
    request3 = GetPositionIKRequest()
    request3.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request3.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request3.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request3.ik_request.pose_stamped.pose.position.x = 0.358
    request3.ik_request.pose_stamped.pose.position.y = 0.655
    request3.ik_request.pose_stamped.pose.position.z = 0.1    
    request3.ik_request.pose_stamped.pose.orientation.x = 0.0
    request3.ik_request.pose_stamped.pose.orientation.y = 1.0
    request3.ik_request.pose_stamped.pose.orientation.z = 0.0
    request3.ik_request.pose_stamped.pose.orientation.w = 0.0
        
    # Construct the 4th request
    request4 = GetPositionIKRequest()
    request4.ik_request.group_name = "right_arm"

    # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
    link = "right_gripper_tip"

    request4.ik_request.ik_link_name = link
    # request.ik_request.attempts = 20
    request4.ik_request.pose_stamped.header.frame_id = "base"
    
    # Set the desired orientation for the end effector HERE
    request4.ik_request.pose_stamped.pose.position.x = 0.358
    request4.ik_request.pose_stamped.pose.position.y = 0.655
    request4.ik_request.pose_stamped.pose.position.z = 0.1      
    request4.ik_request.pose_stamped.pose.orientation.x = 0.0
    request4.ik_request.pose_stamped.pose.orientation.y = 1.0
    request4.ik_request.pose_stamped.pose.orientation.z = 0.0
    request4.ik_request.pose_stamped.pose.orientation.w = 0.0

    try:
        # Send the request to the service
        response = compute_ik(request)
        
        # Print the response HERE
        print(response)
        
        group = MoveGroupCommander("right_arm")
        
        # Setting position and orientation target
        group.set_pose_target(request.ik_request.pose_stamped)

        # TRY THIS
        # Setting just the position without specifying the orientation
        #group.set_position_target([0.5, 0.5, 0.0])

        # Plan IK
        plan = group.plan()
        user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
        
        # Execute IK if safe
        if user_input == 'y':
            print('Opening...')
            right_gripper.open()
            rospy.sleep(1.0)
            group.execute(plan[1])
            

        response2 = compute_ik(request2)
        print(response2)
        group2 = MoveGroupCommander("right_arm")
        group2.set_pose_target(request2.ik_request.pose_stamped)
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
            response3 = compute_ik(request3)
            print(response3)
            group3 = MoveGroupCommander("right_arm")
            group3.set_pose_target(request3.ik_request.pose_stamped)
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
            print("dropping block off")
            response4 = compute_ik(request4)
            print(response4)
            group4 = MoveGroupCommander("right_arm")
            group4.set_pose_target(request4.ik_request.pose_stamped)
            plan4 = group4.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            # Execute IK if safe
            if user_input == 'y':
                group4.execute(plan4[1])
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)
                print('Done!')

        # rospack = rospkg.RosPack()
        # path = rospack.get_path('sawyer_full_stack')
        # launch_path = path + '/launch/custom_sawyer_tuck.launch'
        # uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(uuid)
        # launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_path])
        # launch.start()


            
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

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
    # main()
