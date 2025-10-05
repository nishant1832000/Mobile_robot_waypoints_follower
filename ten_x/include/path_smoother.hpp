#ifndef PATH_SMOOTHER_HPP
#define PATH_SMOOTHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <iostream>

/// ============== Simple Natural Cubic Spline (1D) ==============
class CubicSpline
{
public:
    CubicSpline() = default;
    CubicSpline(const std::vector<double>& t_in, const std::vector<double>& v_in);

    void set_points(const std::vector<double>& t_in, const std::vector<double>& v_in);
    double operator()(double s) const;
    bool is_valid() const;

private:
    std::vector<double> t_, v_;
    std::vector<double> a_, b_, c_, d_; // per-segment coefficients
    bool valid_ = false;

    void compute_coeffs();
    int find_segment(double s) const;
};

/// ============== ROS2 Node ==============
class SplinePathPlanner : public rclcpp::Node
{
public:
    SplinePathPlanner();

private:
    struct Point2D {
        double x, y;
        Point2D(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
    };

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Point2D> waypoints_;

    void initialize_waypoints();
    void timer_callback();
    void publish_waypoints_markers();
    void compute_and_publish_spline_path();
    std::vector<Point2D> compute_natural_cubic_spline(const std::vector<Point2D>& points, double step);
};

#endif // PATH_SMOOTHER_HPP
