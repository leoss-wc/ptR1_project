#!/usr/bin/env python
import rospy
import tf

def lookup_and_print(listener, parent_frame, child_frame):
    try:
        # ใช้ rospy.Time(0) เพื่อขอ TF ล่าสุดที่มี ไม่ต้องอิงเวลา stamp
        (trans, rot) = listener.lookupTransform(parent_frame, child_frame, rospy.Time(0))

        rospy.loginfo("TF [%s → %s]", child_frame, parent_frame)
        rospy.loginfo("  Translation: x=%.3f, y=%.3f, z=%.3f", *trans)
        rospy.loginfo("  Rotation (quat): x=%.3f, y=%.3f, z=%.3f, w=%.3f", *rot)

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.logwarn("TF lookup failed [%s → %s]: %s", child_frame, parent_frame, str(e))

def main():
    rospy.init_node('imu_tf_monitor')
    listener = tf.TransformListener()

    rate = rospy.Rate(1.0)  # 1 Hz
    rospy.loginfo("🔍 Monitoring TF: imu_link → base_link and mag_link → base_link")

    while not rospy.is_shutdown():
        lookup_and_print(listener, "base_link", "imu_link")
        lookup_and_print(listener, "base_link", "mag_link")
        rate.sleep()

if __name__ == '__main__':
    main()
