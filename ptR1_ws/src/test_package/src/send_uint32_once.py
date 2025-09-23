#!/usr/bin/env python

import rospy
from std_msgs.msg import UInt32

def main():
    rospy.init_node('uint32_sender_once', anonymous=True)
    pub = rospy.Publisher('/rb/cm/ed', UInt32, queue_size=1)
    rospy.sleep(0.5)  # รอ publisher พร้อม

    msg = UInt32()
    msg.data = 0x0A000001
    pub.publish(msg)
    rospy.loginfo(f"Published: {hex(msg.data)} to /rb/cm/ed")

if __name__ == '__main__':
    main()
