#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

void publishStaticTF(const std::string& parent, const std::string& child,
                     double x, double y, double z,
                     double roll, double pitch, double yaw) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent;
    transformStamped.child_frame_id = child;

    transformStamped.transform.translation.x = x;
    transformStamped.transform.translation.y = y;
    transformStamped.transform.translation.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);  // RPY = Roll, Pitch, Yaw (radians)
    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();

    static_broadcaster.sendTransform(transformStamped);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_tf_broadcaster");
    ros::NodeHandle nh;
    ros::Duration(0.5).sleep();

    // IMU: วางตรง, ไม่หมุน
    publishStaticTF("base_link", "imu_link", 0.09, 0.01, 0.24, 0.0, 0.0, 0.0);

    // Compass: วางตรง, ไม่หมุน
    publishStaticTF("base_link", "mag_link", 0.03, 0.02, 0.24, 0.0, 0.0, 0.0);
    
    ros::spin();
    return 0;
}
