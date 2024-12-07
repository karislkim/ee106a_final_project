import rospy
from geometry_msgs.msg import Point

class TopicMonitor:
    def __init__(self):
        rospy.init_node("topic_monitor")
        
        # Parameters
        self.timeout_duration = 5.0  # Seconds
        self.last_received_time = rospy.Time.now()
        
        # Subscriber to the topic
        self.sub = rospy.Subscriber("/goal_point", Point, self.callback)
        
        # Timer to check for timeouts
        self.timer = rospy.Timer(rospy.Duration(1), self.check_timeout)
        
        rospy.loginfo("Topic monitor initialized.")

    def callback(self, msg):
        # Update the timestamp whenever a message is received
        rospy.loginfo(f"Received message: {msg}")
        self.last_received_time = rospy.Time.now()

    def check_timeout(self, event):
        # Check if timeout has occurred
        time_since_last_msg = (rospy.Time.now() - self.last_received_time).to_sec()
        
        if time_since_last_msg > self.timeout_duration:
            rospy.logwarn("No messages received on /goal_point within timeout duration!")
            self.handle_timeout()
        else:
            rospy.loginfo("Messages are being published.")

    def handle_timeout(self):
        # Define what to do when timeout occurs
        rospy.loginfo("Executing fallback behavior...")

if __name__ == "__main__":
    try:
        TopicMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
