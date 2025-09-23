#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry

class MecanumOdometryNode:
    def __init__(self):
        rospy.init_node('mecanum_velocity_node')

        # Robot parameters
        self.r = rospy.get_param("~wheel_radius", 0.04)
        self.l1 = rospy.get_param("~l1", 0.1075)
        self.l2 = rospy.get_param("~l2", 0.0825)
        self.PPR = rospy.get_param("~ppr", 660.0)
        self.dt = rospy.get_param("~update_dt", 0.02)

        # Encoder values
        self.enc = {'FL': None, 'FR': None, 'RL': None, 'RR': None}
        self.prev = {'FL': None, 'FR': None, 'RL': None, 'RR': None}

        # Compensation scale factors (default = 1.0)
        self.scale = {
            'FL': rospy.get_param("~scale_FL", 1.0000),
            'FR': rospy.get_param("~scale_FR", 0.9993),
            'RL': rospy.get_param("~scale_RL", 0.9988),
            'RR': rospy.get_param("~scale_RR", 0.9974)
        }

        enc_topics = {
            'FL': rospy.get_param("~encoder1_topic", "encoder1"),
            'FR': rospy.get_param("~encoder2_topic", "encoder2"),
            'RL': rospy.get_param("~encoder3_topic", "encoder3"),
            'RR': rospy.get_param("~encoder4_topic", "encoder4")
        }

        for key in ['FL', 'FR', 'RL', 'RR']:
            rospy.Subscriber(enc_topics[key], Int32, lambda msg, k=key: self.enc_cb(k, msg))

        self.odom_pub = rospy.Publisher("/wheel_odom_raspi", Odometry, queue_size=10)

        rospy.Timer(rospy.Duration(self.dt), self.update_velocity)

    def enc_cb(self, key, msg):
        self.enc[key] = msg.data
        if self.prev[key] is None:
            self.prev[key] = self.enc[key]

    def update_velocity(self, event):
        if None in self.enc.values():
            return

        # delta pulse
        delta = {
            key: (self.enc[key] - self.prev[key]) * self.scale[key]
            for key in self.enc
        }

        for key in self.enc:
            self.prev[key] = self.enc[key]

        # pulse → angular velocity (rad/s)
        def pulse_to_rad(pulse): return (pulse / self.PPR) * 2 * 3.14159265 / self.dt
        w = {key: pulse_to_rad(delta[key]) for key in delta}

        # Mecanum inverse kinematics
        vx = (self.r / 4.0) * (w['FL'] + w['FR'] + w['RL'] + w['RR'])
        vy = (self.r / 4.0) * (-w['FL'] + w['FR'] + w['RL'] - w['RR'])
        omega = (self.r / (4.0 * (self.l1 + self.l2))) * (-w['FL'] + w['FR'] - w['RL'] + w['RR'])

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega

        odom.twist.covariance = [
            0.01, 0,    0, 0, 0, 0,
            0,    0.01, 0, 0, 0, 0,
            0,    0,    999, 0, 0, 0,
            0,    0,    0, 999, 0, 0,
            0,    0,    0, 0, 999, 0,
            0,    0,    0, 0, 0, 0.1
        ]

        self.odom_pub.publish(odom)

        # Optional: debug
        rospy.logdebug(f"vx={vx:.3f}, vy={vy:.3f}, omega={omega:.3f}")

if __name__ == '__main__':
    try:
        MecanumOdometryNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
