#include "path_smoother.hpp"

/// ================= CubicSpline Implementation =================
CubicSpline::CubicSpline(const std::vector<double>& t_in, const std::vector<double>& v_in)
{
    set_points(t_in, v_in);
}

void CubicSpline::set_points(const std::vector<double>& t_in, const std::vector<double>& v_in)
{
    t_ = t_in;
    v_ = v_in;
    int n = static_cast<int>(t_.size());
    if (n < 2 || v_.size() != t_.size()) {
        valid_ = false;
        return;
    }
    compute_coeffs();
    valid_ = true;
}

double CubicSpline::operator()(double s) const
{
    if (!valid_) return 0.0;
    int idx = find_segment(s);
    double dt = s - t_[idx];
    return a_[idx] + b_[idx]*dt + c_[idx]*dt*dt + d_[idx]*dt*dt*dt;
}

bool CubicSpline::is_valid() const { return valid_; }

void CubicSpline::compute_coeffs()
{
    int n = static_cast<int>(t_.size());
    int m = n - 1;
    a_.assign(m, 0.0);
    b_.assign(m, 0.0);
    c_.assign(m+1, 0.0);
    d_.assign(m, 0.0);

    for (int i = 0; i < m; ++i) a_[i] = v_[i];

    std::vector<double> h(m);
    for (int i = 0; i < m; ++i) h[i] = t_[i+1] - t_[i];

    if (m == 1) {
        b_[0] = (v_[1] - v_[0]) / h[0];
        c_.assign(2, 0.0);
        d_[0] = 0.0;
        return;
    }

    std::vector<double> alpha(n, 0.0);
    for (int i = 1; i <= n-2; ++i) {
        alpha[i] = 3.0*((v_[i+1]-v_[i])/h[i] - (v_[i]-v_[i-1])/h[i-1]);
    }

    std::vector<double> l(n, 0.0), mu(n, 0.0), z(n, 0.0);
    l[0] = 1.0; mu[0] = 0.0; z[0] = 0.0;

    for (int i = 1; i <= n-2; ++i) {
        l[i] = 2.0*(t_[i+1]-t_[i-1]) - h[i-1]*mu[i-1];
        mu[i] = h[i]/l[i];
        z[i] = (alpha[i] - h[i-1]*z[i-1])/l[i];
    }

    l[n-1] = 1.0;
    z[n-1] = 0.0;
    c_[n-1] = 0.0;

    for (int j = n-2; j >= 0; --j) {
        c_[j] = z[j] - mu[j]*c_[j+1];
        b_[j] = (v_[j+1]-v_[j])/h[j] - h[j]*(c_[j+1]+2.0*c_[j])/3.0;
        d_[j] = (c_[j+1]-c_[j])/(3.0*h[j]);
    }
}

int CubicSpline::find_segment(double s) const
{
    int n = static_cast<int>(t_.size());
    if (s <= t_.front()) return 0;
    if (s >= t_.back()) return n - 2;
    int lo = 0, hi = n - 1;
    while (hi - lo > 1) {
        int mid = (lo + hi) / 2;
        if (t_[mid] <= s) lo = mid;
        else hi = mid;
    }
    return lo;
}

/// ================= SplinePathPlanner Implementation =================
SplinePathPlanner::SplinePathPlanner() : Node("spline_path_planner")
{
    waypoints_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints_markers", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/spline_path", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200),
        std::bind(&SplinePathPlanner::timer_callback, this));

    initialize_waypoints();
    publish_waypoints_markers();

    RCLCPP_INFO(this->get_logger(), "Spline Path Planner node started");
}

void SplinePathPlanner::initialize_waypoints()
{
    waypoints_.clear();
    waypoints_.emplace_back(0.0, 0.0);
    waypoints_.emplace_back(1.0, 2.0);
    waypoints_.emplace_back(2.0, 3.0);
    waypoints_.emplace_back(3.0, 3.0);
    waypoints_.emplace_back(4.0, 5.0);
    waypoints_.emplace_back(6.0, 4.0);
}

void SplinePathPlanner::timer_callback()
{
    compute_and_publish_spline_path();
}

void SplinePathPlanner::publish_waypoints_markers()
{
    visualization_msgs::msg::MarkerArray marker_array;

    for (size_t i = 0; i < waypoints_.size(); ++i) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoints_[i].x;
        marker.pose.position.y = waypoints_[i].y;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
        marker.color.r = 1.0; marker.color.a = 1.0;

        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker text_marker = marker;
        text_marker.ns = "waypoints_text";
        text_marker.id = static_cast<int>(i + waypoints_.size());
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z = 0.3;
        text_marker.scale.z = 0.2;
        text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0;
        text_marker.text = "WP" + std::to_string(i);

        marker_array.markers.push_back(text_marker);
    }

    waypoints_pub_->publish(marker_array);
}

void SplinePathPlanner::compute_and_publish_spline_path()
{
    if (waypoints_.size() < 2) {
        RCLCPP_WARN(this->get_logger(), "Need at least 2 waypoints to compute spline");
        return;
    }

    double step = 0.0008;
    auto spline_points = compute_natural_cubic_spline(waypoints_, step);

    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odom";
    path_msg.header.stamp = this->now();

    for (const auto& p : spline_points) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp = this->now();
        pose.pose.position.x = p.x;
        pose.pose.position.y = p.y;
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
}

std::vector<SplinePathPlanner::Point2D> SplinePathPlanner::compute_natural_cubic_spline(
    const std::vector<Point2D>& points, double step)
{
    std::vector<Point2D> spline_points;
    size_t N = points.size();
    if (N < 2) return spline_points;

    std::vector<double> t(N, 0.0);
    for (size_t i = 1; i < N; ++i) {
        double dx = points[i].x - points[i-1].x;
        double dy = points[i].y - points[i-1].y;
        t[i] = t[i-1] + std::sqrt(dx*dx + dy*dy);
    }

    double total_length = t.back();
    if (total_length <= 1e-9) {
        for (size_t i = 0; i < N; ++i) t[i] = static_cast<double>(i);
    }

    std::vector<double> t_in(N), x_in(N), y_in(N);
    for (size_t i = 0; i < N; ++i) {
        t_in[i] = t[i];
        x_in[i] = points[i].x;
        y_in[i] = points[i].y;
    }

    CubicSpline cs_x(t_in, x_in);
    CubicSpline cs_y(t_in, y_in);
    if (!cs_x.is_valid() || !cs_y.is_valid()) {
        RCLCPP_ERROR(this->get_logger(), "Cubic spline creation failed");
        return spline_points;
    }

    double u = 0.0;
    while (u <= 1.0 + 1e-12) {
        double s = u * t_in.back();
        spline_points.emplace_back(cs_x(s), cs_y(s));
        u += step;
    }

    spline_points.emplace_back(points.back().x, points.back().y);

    std::vector<Point2D> unique_points;
    for (const auto& p : spline_points) {
        if (unique_points.empty() ||
            std::hypot(p.x - unique_points.back().x, p.y - unique_points.back().y) > 1e-6)
            unique_points.push_back(p);
    }

    return unique_points;
}

/// ================= main =================
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SplinePathPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
