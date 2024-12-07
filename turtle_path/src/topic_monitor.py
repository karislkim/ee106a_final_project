import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class TopicMonitor:
    def __init__(self):
        rospy.init_node("topic_monitor")
        
        # Parameters
        self.timeout_duration = 2.0  # Seconds
        self.last_received_time = rospy.Time.now()
        
        # Subscriber to the topic
        self.sub = rospy.Subscriber("/goal_point", Point, self.callback)
        self.is_pub = rospy.Publisher("topic_monitor", Bool, queue_size=10)
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
        not_publishing_to_gpt = time_since_last_msg > self.timeout_duration
        if not_publishing_to_gpt:
            rospy.logwarn("No messages received on /goal_point within timeout duration!")
            self.is_pub.publish(not_publishing_to_gpt)
            print(f"publishing_to_gpt: {not_publishing_to_gpt}")
            self.handle_timeout()
        else:
            self.is_pub.publish(publishing_to_gpt)
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
