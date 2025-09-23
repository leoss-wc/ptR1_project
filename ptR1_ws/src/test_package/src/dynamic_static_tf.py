#!/usr/bin/env python
import rospy
import tf2_ros
import geometry_msgs.msg
from sensor_msgs.msg import Imu\

br = None
imu_tf = None
mag_tf = None

def create_tf(child, parent, x, y, z):
    tf = geometry_msgs.msg.TransformStamped()
    tf.header.frame_id = parent     
    tf.child_frame_id = child         
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z
    tf.transform.rotation.x = 0.0
    tf.transform.rotation.y = 0.0
    tf.transform.rotation.z = 0.0
    tf.transform.rotation.w = 1.0
    return tf

def imu_callback(msg):
    stamp = msg.header.stamp
    stamp = msg.header.stamp

    imu_tf = create_tf("imu_link", "base_link", 0.09, 0.01, 0.24)
    imu_tf.header.stamp = stamp

    mag_tf = create_tf("mag_link", "base_link", 0.03, 0.02, 0.24)
    mag_tf.header.stamp = stamp

    br.sendTransform([imu_tf, mag_tf])

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')

    br = tf2_ros.TransformBroadcaster()

    imu_tf = create_tf("imu_link", "base_link", 0.09, 0.01, 0.24)
    mag_tf = create_tf("mag_link", "base_link", 0.03, 0.02, 0.24)

    rospy.Subscriber("/imu/data_raw", Imu, imu_callback)
    rospy.spin()
