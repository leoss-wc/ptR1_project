#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt32, UInt16, UInt8

class CommandHandler:
    def __init__(self):
        rospy.init_node('command_node', anonymous=True)
        self.pub_motor = rospy.Publisher('/arduino/drive', UInt16, queue_size=100)
        self.pub_servo = rospy.Publisher('/arduino/servo', UInt8, queue_size=100)
        self.pub_edit = rospy.Publisher('/arduino/edit', UInt32, queue_size=100)
        self.manual_drive_data = None  # เก็บคำสั่งจาก /rb/cm/dr
        self.auto_drive_data = None    # เก็บคำสั่งจาก /cmd_vel
        self.mode = 1
        rospy.loginfo(f"Initial Mode: {'MANUAL' if self.mode == 1 else 'AUTO'}")


        rospy.Subscriber('/rb/cm/dr', UInt16, self.callback_dr)
        rospy.Subscriber('/rb/cm/sv', UInt16, self.callback_sv)
        rospy.Subscriber('/rb/cm/ed', UInt32, self.callback_ed)

        rospy.loginfo("Node command_handler initialized and running...")  

    def process_message(self, msg, pub, max_value, topic_name):
        try:
            if not isinstance(msg.data, int):
                rospy.logerr(f"Invalid data type received on {topic_name}: {msg.data}")
                return
            data_int = msg.data
            if 0 <= data_int <= max_value:
                pub.publish(data_int)
                rospy.loginfo(f"Published {data_int} to {topic_name}")
            else:
                rospy.logwarn(f"Value out of range for {topic_name}: {data_int}")
        except ValueError:
            rospy.logerr(f"Invalid data received on {topic_name}: {msg.data} ")

    def callback_dr(self, msg):
        if self.mode == 1:
            try:
                self.manual_drive_data = msg.data
                self.pub_motor.publish(self.manual_drive_data)
                rospy.loginfo(f"MANUAL: Published to /arduino/drive: {self.manual_drive_data}")
            except ValueError:
                rospy.logerr(f"Invalid data received on /rb/cm/dr: {msg.data}")

    def callback_sv(self, msg):
        self.process_message(msg, self.pub_servo, 0xFF, '/arduino/servo')

    def callback_ed(self, msg):
        try:
            data = int(msg.data)
            value = data & 0xFFFFFF
            if (data >> 24) & 0xFF == 0x05:
                if value == 0: #AUTO
                    self.mode = 0 #set mode
                    self.process_message(msg, self.pub_edit, 0xFFFFFFFF, '/arduino/edit')
                elif value == 1: #MANUAL
                    self.mode = 1  #set mode
                    self.process_message(msg, self.pub_edit, 0xFFFFFFFF, '/arduino/edit')
                else:
                    rospy.logwarn(f"Invalid mode value received on /rb/cm/ed: {value}")
            else:
                self.process_message(msg, self.pub_edit, 0xFFFFFFFF, '/arduino/edit')
        except (ValueError, AttributeError):
            rospy.logerr(f"Invalid data received on /rb/cm/ed: {msg.data}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        command_node = CommandHandler()
        command_node.run()
    except rospy.ROSInterruptException:
        pass
