#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class TransferDataNode:
    def __init__(self):
        rospy.init_node('transfer_data_node', anonymous=True)
        self.pub = rospy.Publisher('/wheel_odom', Odometry, queue_size=10)
        rospy.Subscriber('/robot_velocity', Twist, self.twist_callback)

    def twist_callback(self, msg):
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.twist.twist = msg

        # (optional) เพิ่ม covariance
        odom.twist.covariance = [
            0.01, 0, 0, 0, 0, 0,
            0, 0.01, 0, 0, 0, 0,
            0, 0, 0.1, 0, 0, 0,
            0, 0, 0, 0.1, 0, 0,
            0, 0, 0, 0, 0.1, 0,
            0, 0, 0, 0, 0, 0.2
        ]

        self.pub.publish(odom)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        TransferDataNode().run()
    except rospy.ROSInterruptException:
        pass
