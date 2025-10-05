#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

def main():
    rospy.init_node('pose_republisher')

    # สร้าง TF buffer และ listener
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    # สร้าง Publisher ไปยัง Topic ใหม่ของเรา
    pose_publisher = rospy.Publisher('/robot_pose_sample', PoseStamped, queue_size=1)

    rate = rospy.Rate(3) # 3 Hz

    while not rospy.is_shutdown():
        try:
            # ขอ Transform ล่าสุดจาก map -> base_footprint
            transform = tf_buffer.lookup_transform('map', 'base_link', rospy.Time(0))

            # สร้างข้อความ PoseStamped
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.position.x = transform.transform.translation.x
            pose_msg.pose.position.y = transform.transform.translation.y
            pose_msg.pose.position.z = transform.transform.translation.z
            
            pose_msg.pose.orientation.x = transform.transform.rotation.x
            pose_msg.pose.orientation.y = transform.transform.rotation.y
            pose_msg.pose.orientation.z = transform.transform.rotation.z
            pose_msg.pose.orientation.w = transform.transform.rotation.w

            # ประกาศข้อความออกไป
            pose_publisher.publish(pose_msg)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "Could not get transform from 'map' to 'base_footprint': %s" % e)
            continue

        rate.sleep()

if __name__ == '__main__':
    main()