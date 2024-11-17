import rospy
from geometry_msgs.msg import Twist
import time
import numpy as np

def move_rectangle():
    rospy.init_node('turtlebot_rectangle_motion', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  

    # Rectangle dimensions
    forward_speed = 0.1  
    turn_speed = 0.5 
    length = 1.4  
    width = 0.15  

    # Move in rectangle
    while not rospy.is_shutdown():
        move_straight(forward_speed, length, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)

        move_straight(forward_speed, width, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        move_straight(forward_speed, length, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        move_straight(forward_speed, width, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        rospy.loginfo("Completed one rectangle path.")
        rospy.sleep(2) 

def move_straight(speed, distance, cmd_vel_pub):
    # Move forward 
    move_cmd = Twist()
    move_cmd.linear.x = speed
    travel_time = distance / speed
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < travel_time:
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(0.1)  

    # Stop moving
    move_cmd.linear.x = 0
    cmd_vel_pub.publish(move_cmd)

def turn_90_degrees(turn_speed, cmd_vel_pub):
    # Turn 90 degrees 
    turn_cmd = Twist()
    turn_cmd.angular.z = turn_speed
    turn_time = 0.5*np.pi / turn_speed 
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < turn_time:
        cmd_vel_pub.publish(turn_cmd)
        rospy.sleep(0.1) 

    # Stop turning
    turn_cmd.angular.z = 0
    cmd_vel_pub.publish(turn_cmd)

if __name__ == '__main__':
    try:
        move_rectangle()
    except rospy.ROSInterruptException:
        pass
