#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Bool
import tf
import math

class PatrolNavigator:
    def __init__(self):
        rospy.init_node('patrol_navigator')

        # ROS interface
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.status_pub = rospy.Publisher('/patrol_status', Bool, queue_size=10)

        rospy.Subscriber('/patrol_path', PoseArray, self.path_callback)
        rospy.Subscriber('/stop_patrol', Bool, self.stop_callback)
        rospy.Subscriber('/rb/navigation/goal', PoseStamped, self.goal_callback)

        self.tf_listener = tf.TransformListener()
        self.goal_tolerance = 0.3  # m

        # Navigation state
        self.path = []
        self.current_index = 0
        self.active = False
        self.reached_goal = False
        self.single_goal_mode = False
        self.current_goal = None

        # Timer loop
        self.timer = rospy.Timer(rospy.Duration(1.0), self.control_loop)
        rospy.loginfo("🧭 patrol_navigator_node started.")
        rospy.spin()

    def goal_callback(self, msg):
        # ⚠️ รับ goal จาก /move_base_simple/goal → ใช้เดินจุดเดียว
        rospy.loginfo("📍 Received single goal from RViz or Electron")
        self.active = True
        self.single_goal_mode = True
        self.current_goal = msg.pose
        self.status_pub.publish(True)

    def path_callback(self, msg):
        if self.active:
            rospy.logwarn("⚠️ Already navigating. Stop first to reset path.")
            return
        self.path = msg.poses
        self.current_index = 0
        self.active = True
        self.reached_goal = False
        self.single_goal_mode = False
        rospy.loginfo("📥 Received patrol path with %d points", len(self.path))
        self.status_pub.publish(True)

    def stop_callback(self, msg):
        if msg.data:
            rospy.loginfo("🛑 Stopping patrol.")
            self.active = False
            self.path = []
            self.current_goal = None
            self.status_pub.publish(False)

    def control_loop(self, event):
        if not self.active:
            return

        try:
            now = rospy.Time(0)
            self.tf_listener.waitForTransform("map", "base_link", now, rospy.Duration(0.5))
            (trans, rot) = self.tf_listener.lookupTransform("map", "base_link", now)
            robot_x, robot_y = trans[0], trans[1]
        except Exception as e:
            rospy.logwarn("TF error: %s", str(e))
            return

        # --- Single Goal Mode ---
        if self.single_goal_mode and self.current_goal:
            dist = self._distance(self.current_goal.position.x, self.current_goal.position.y, robot_x, robot_y)
            if dist < self.goal_tolerance:
                rospy.loginfo("✅ Reached single goal.")
                self.active = False
                self.current_goal = None
                self.status_pub.publish(False)
            else:
                self.publish_goal(self.current_goal)
            return

        # --- Patrol Mode ---
        if self.current_index >= len(self.path):
            rospy.loginfo("🎉 Completed patrol path.")
            self.active = False
            self.status_pub.publish(False)
            return

        target = self.path[self.current_index].position
        dist = self._distance(target.x, target.y, robot_x, robot_y)

        if dist < self.goal_tolerance:
            rospy.loginfo("✅ Reached point %d", self.current_index + 1)
            self.current_index += 1
            rospy.sleep(0.5)
        else:
            self.publish_goal(self.path[self.current_index])

    def publish_goal(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose = pose
        self.goal_pub.publish(msg)
        rospy.loginfo("🚩 Sending goal to: (%.2f, %.2f)", pose.position.x, pose.position.y)

    def _distance(self, x1, y1, x2, y2):
        return math.hypot(x1 - x2, y1 - y2)


if __name__ == '__main__':
    try:
        PatrolNavigator()
    except rospy.ROSInterruptException:
        pass
