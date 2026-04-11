#ifndef LOCALIZATION_HPP
#define LOCALIZATION_HPP

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode();

private:
    void jointCallback(const sensor_msgs::msg::JointState & msg);

    void updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt, rclcpp::Time stamp);
    void publishOdometry();
    void publishTransform();

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_subscriber_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;

    // Transformations
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Odometry odometry_;
    rclcpp::Time last_time_;
};

#endif // LOCALIZATION_HPP
