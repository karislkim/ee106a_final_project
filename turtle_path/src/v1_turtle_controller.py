import rospy
from geometry_msgs.msg import Twist
import time

def move_rectangle():
    rospy.init_node('turtlebot_rectangle_motion', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz update rate

    # Rectangle parameters
    forward_speed = 0.1  # Speed while moving forward (m/s)
    turn_speed = 0.5  # Angular speed (rad/s)
    length = 1.4  # Length of the rectangle (meters)
    width = 0.15  # Width of the rectangle (meters)

    # Move the robot in a rectangular path
    while not rospy.is_shutdown():
        # Move forward along the longer side (length)
        move_straight(forward_speed, length, cmd_vel_pub)
        
        # Turn 90 degrees
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        # Move forward along the shorter side (width)
        move_straight(forward_speed, width, cmd_vel_pub)
        
        # Turn 90 degrees again
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        # Repeat the rectangle
        move_straight(forward_speed, length, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        move_straight(forward_speed, width, cmd_vel_pub)
        turn_90_degrees(turn_speed, cmd_vel_pub)
        
        rospy.loginfo("Completed one rectangle path.")
        rospy.sleep(2)  # Pause briefly before restarting

def move_straight(speed, distance, cmd_vel_pub):
    # Move forward for a set distance
    move_cmd = Twist()
    move_cmd.linear.x = speed

    # Time to travel the given distance (assuming speed = distance/time)
    travel_time = distance / speed
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < travel_time:
        cmd_vel_pub.publish(move_cmd)
        rospy.sleep(0.1)  # Publish at 10Hz

    # Stop moving
    move_cmd.linear.x = 0
    cmd_vel_pub.publish(move_cmd)

def turn_90_degrees(turn_speed, cmd_vel_pub):
    # Turn 90 degrees (in radians)
    turn_cmd = Twist()
    turn_cmd.angular.z = turn_speed

    # Time to turn 90 degrees (assuming speed = distance/time and angular distance for 90 degrees = pi/2 radians)
    turn_time = 1.57 / turn_speed  # 1.57 rad = 90 degrees
    start_time = rospy.get_time()

    while rospy.get_time() - start_time < turn_time:
        cmd_vel_pub.publish(turn_cmd)
        rospy.sleep(0.1)  # Publish at 10Hz

    # Stop turning
    turn_cmd.angular.z = 0
    cmd_vel_pub.publish(turn_cmd)

if __name__ == '__main__':
    try:
        move_rectangle()
    except rospy.ROSInterruptException:
        pass