import rospy
from std_msgs.msg import UInt32  # ใช้ UInt32 แทน String

def data_sender():
    speed = 60

    rospy.init_node('data_sender_node', anonymous=True)
    pub = rospy.Publisher('arduino_command', UInt32, queue_size=10)
    rate = rospy.Rate(0.5)  # 1 Hz

    data_patterns = [
        0x010100, 0x010200, 0x010300, 0x010400, 0x010500,
        0x010600, 0x010700, 0x010800, 0x010900, 0x010A00
    ]

    index = 0
    while not rospy.is_shutdown():
        # Cycle through the data patterns
        base_message = data_patterns[index % len(data_patterns)]
        message = base_message + speed  # รวมค่าพื้นฐานกับ speed (speed = 100)

        rospy.loginfo(f"Sending: {hex(message)}")  # แสดงค่าข้อมูลในรูปแบบ Hexadecimal
        pub.publish(message)  # ส่งข้อมูลในรูปแบบ UInt32

        index += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        data_sender()
    except rospy.ROSInterruptException:
        pass
