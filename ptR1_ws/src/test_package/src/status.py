#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import String, UInt32, UInt16, UInt8

class Status:
    def __init__(self):
        rospy.init_node('status_node', anonymous=True)
        rospy.Subscriber('/arduino/status', UInt32, self.callback)
        self.status_pub = rospy.Publisher('/rb/st', UInt32, queue_size=20)
    def callback(self, data):
        rospy.loginfo(f"Received status: {data.data}")
        self.status_pub.publish(data.data)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        ST = Status()
        ST.run()
    except rospy.ROSInterruptException:
        pass
