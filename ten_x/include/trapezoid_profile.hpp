#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <vector>
#include <cmath>

class TrapezoidVelocityNode : public rclcpp::Node
{
public:
    TrapezoidVelocityNode();

private:
    struct TrajectoryPoint {
        double x;
        double y;
        double v;
        double t;
    };

    // --- Subscribers & Publishers ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // --- Parameters & State ---
    double v_max_;
    double a_max_;
    std::vector<TrajectoryPoint> trajectory_;
    size_t traj_index_;
    double robot_x_, robot_y_, robot_yaw_;
    std::vector<geometry_msgs::msg::PoseStamped> last_path_;
    bool initial_yaw_matching_;
    double target_initial_yaw_;

    // --- Callbacks ---
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void controlTimer();

    // --- Helpers ---
    void computeTrajectoryFromPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses);
};
