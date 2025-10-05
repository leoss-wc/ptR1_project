#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# ข้อความอธิบายวิธีใช้แบบใหม่
msg = """
Control Your Mecanum Robot (Tap Control)!
---------------------------
Press keys to increase/decrease velocity step-by-step.
The robot will hold its current velocity until a new command is given.

Moving around:
   q    w    e
   a    s    d
        x

w/x : increase/decrease forward/backward velocity
a/d : increase/decrease strafe left/right velocity
q/e : increase/decrease turning velocity

space key, s : EMERGENCY STOP (set all velocities to 0)

CTRL-C to quit
"""

# --- ค่าคงที่สำหรับการควบคุม ---
# ค่าความเร็วที่จะเพิ่ม/ลดในแต่ละครั้งที่กด
LINEAR_VEL_STEP = 0.05
ANGULAR_VEL_STEP = 0.1

# ความเร็วสูงสุด
MAX_LINEAR_VEL = 1.0
MAX_ANGULAR_VEL = 1.5

def getKey():
    """ฟังก์ชันสำหรับดึงค่าการกดปุ่มจากคีย์บอร์ด"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def print_vels(vx, vy, vth):
    """ฟังก์ชันสำหรับแสดงผลความเร็วปัจจุบัน"""
    print(f"\rCurrent Velocity -> Linear(x,y): ({vx:.2f}, {vy:.2f}) | Angular(z): {vth:.2f}      ", end="")


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('mecanum_teleop_tap')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    # ตัวแปรเก็บค่าความเร็วปัจจุบันของหุ่นยนต์
    vx = 0.0  # linear.x
    vy = 0.0  # linear.y
    vth = 0.0 # angular.z

    try:
        print(msg)
        print_vels(vx, vy, vth)

        while not rospy.is_shutdown():
            key = getKey()

            # --- ตรรกะการควบคุมความเร็ว ---
            if key:
                if key == 'w':
                    vx += LINEAR_VEL_STEP
                elif key == 's':
                    vx -= LINEAR_VEL_STEP
                elif key == 'a':
                    vy += LINEAR_VEL_STEP
                elif key == 'd':
                    vy -= LINEAR_VEL_STEP
                elif key == 'q':
                    vth += ANGULAR_VEL_STEP
                elif key == 'e':
                    vth -= ANGULAR_VEL_STEP
                elif key == ' ' or key == 'g':
                    vx, vy, vth = 0.0, 0.0, 0.0
                elif (key == '\x03'): # CTRL-C
                    break
                
                # --- จำกัดความเร็วไม่ให้เกินค่าสูงสุด ---
                vx = max(min(vx, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
                vy = max(min(vy, MAX_LINEAR_VEL), -MAX_LINEAR_VEL)
                vth = max(min(vth, MAX_ANGULAR_VEL), -MAX_ANGULAR_VEL)

                print_vels(vx, vy, vth)

            # สร้างและ publish ข้อความ Twist
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = vth
            pub.publish(twist)

            rospy.sleep(0.05) # ลดการทำงานของ CPU

    except Exception as e:
        print(e)

    finally:
        # ส่งคำสั่งหยุดเมื่อจบโปรแกรม
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)