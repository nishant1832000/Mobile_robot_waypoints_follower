#include "trapezoid_profile.hpp"

TrapezoidVelocityNode::TrapezoidVelocityNode()
: Node("trapezoid_velocity_node"),
  v_max_(0.26),
  a_max_(0.01),
  traj_index_(0),
  robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
  initial_yaw_matching_(false), target_initial_yaw_(0.0)
{
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "/spline_path", 10,
        std::bind(&TrapezoidVelocityNode::pathCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&TrapezoidVelocityNode::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Trapezoid Velocity Node started.");
}

void TrapezoidVelocityNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
    if (msg->poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty path!");
        return;
    }

    if (msg->poses == last_path_) {
        RCLCPP_INFO(this->get_logger(), "Received same path, skipping trajectory computation.");
        return;
    }

    last_path_ = msg->poses;
    path_sub_.reset();
    computeTrajectoryFromPath(msg->poses);
}

void TrapezoidVelocityNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robot_x_ = msg->pose.pose.position.x;
    robot_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    RCLCPP_INFO(this->get_logger(), "Received odom info");
}

void TrapezoidVelocityNode::computeTrajectoryFromPath(const std::vector<geometry_msgs::msg::PoseStamped>& poses)
{
    std::vector<double> segment_lengths;
    double total_length = 0.0;

    for (size_t i = 1; i < poses.size(); i++) {
        double dx = poses[i].pose.position.x - poses[i-1].pose.position.x;
        double dy = poses[i].pose.position.y - poses[i-1].pose.position.y;
        double dist = std::hypot(dx, dy);
        segment_lengths.push_back(dist);
        total_length += dist;
    }

    RCLCPP_INFO(this->get_logger(), "Total Path Length: %.2f m", total_length);

    double t_accel = v_max_ / a_max_;
    double d_accel = 0.5 * a_max_ * t_accel * t_accel;
    double d_const = total_length - 2 * d_accel;

    if (d_const < 0) {
        d_accel = total_length / 2.0;
        t_accel = std::sqrt(2 * d_accel / a_max_);
        d_const = 0;
    }

    double s = 0.0;
    double t = 0.0;
    trajectory_.clear();
    std::vector<double> velocity;
    velocity.push_back(0.0);
    double dt = 0.0;

    for (size_t i = 0; i < segment_lengths.size(); i++) {
        double ds = segment_lengths[i];
        s += ds;

        double v;
        if (s < d_accel)
            v = std::sqrt(2 * a_max_ * s);
        else if (s < d_accel + d_const)
            v = v_max_;
        else {
            double d_rem = total_length - s;
            v = std::sqrt(2 * a_max_ * d_rem);
        }

        if (v != velocity.back())
            dt = std::abs(v - velocity.back()) / a_max_;
        else
            dt = ds / v;

        t += dt;
        velocity.push_back(v);

        double x = poses[i].pose.position.x;
        double y = poses[i].pose.position.y;
        std::cout << x << " , " << y << " , " << v << " , " << t << " , " << dt << std::endl;

        trajectory_.push_back({x, y, v, t});
    }

    traj_index_ = 0;
    RCLCPP_INFO(this->get_logger(), "Trajectory computed. Size: %zu", trajectory_.size());

    if (trajectory_.size() > 1) {
        double dy_i = trajectory_[1].y - trajectory_[0].y;
        double dx_i = trajectory_[1].x - trajectory_[0].x;
        target_initial_yaw_ = std::atan2(dy_i, dx_i);
        initial_yaw_matching_ = true;
    } else {
        initial_yaw_matching_ = false;
    }

    if (!timer_) {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&TrapezoidVelocityNode::controlTimer, this));
    }
}

void TrapezoidVelocityNode::controlTimer()
{
    if (initial_yaw_matching_) {
        double error_yaw_i = target_initial_yaw_ - robot_yaw_;
        while (error_yaw_i > M_PI) error_yaw_i -= 2*M_PI;
        while (error_yaw_i < -M_PI) error_yaw_i += 2*M_PI;

        if (std::abs(error_yaw_i) >= 0.001) {
            double omega = 0.5 * error_yaw_i;
            omega = std::clamp(omega, -0.5, 0.5);

            geometry_msgs::msg::TwistStamped cmd;
            cmd.header.stamp = this->now();
            cmd.header.frame_id = "base_link";
            cmd.twist.linear.x = 0.0;
            cmd.twist.angular.z = omega;
            vel_pub_->publish(cmd);
            return;
        } else {
            initial_yaw_matching_ = false;
            RCLCPP_INFO(this->get_logger(), "Initial slope matching successfully.");
        }
    }

    static double prev_yaw_error = 0.0, yaw_error_integral = 0.0;
    static double prev_dist_error = 0.0, dist_error_integral = 0.0;
    double dt = 0.1;

    if (traj_index_ < trajectory_.size()) {
        const auto& target = trajectory_[traj_index_];
        if (traj_index_ > 0) {
            dt = target.t - trajectory_[traj_index_-1].t;
            if (dt <= 0.0) dt = 0.1;
        }

        double dx = target.x - robot_x_;
        double dy = target.y - robot_y_;
        double target_yaw = std::atan2(dy, dx);
        double yaw_error = target_yaw - robot_yaw_;
        while (yaw_error > M_PI) yaw_error -= 2*M_PI;
        while (yaw_error < -M_PI) yaw_error += 2*M_PI;

        double distance = std::hypot(dx, dy);
        double dist_error = distance;

        double Kp_lin = 1.5, Ki_lin = 0.0, Kd_lin = 0.1;
        double Kp_ang = 0.8, Ki_ang = 0.0, Kd_ang = 0.1;

        dist_error_integral += dist_error * dt;
        double dist_error_deriv = (dist_error - prev_dist_error) / dt;
        double v = Kp_lin * dist_error + Ki_lin * dist_error_integral + Kd_lin * dist_error_deriv;
        v = std::clamp(v, 0.0, target.v);
        prev_dist_error = dist_error;

        yaw_error_integral += yaw_error * dt;
        double yaw_error_deriv = (yaw_error - prev_yaw_error) / dt;
        double w = Kp_ang * yaw_error + Ki_ang * yaw_error_integral + Kd_ang * yaw_error_deriv;
        prev_yaw_error = yaw_error;

        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = v;
        cmd.twist.angular.z = w;
        vel_pub_->publish(cmd);

        if (distance < 0.05)
            traj_index_++;
    } else {
        geometry_msgs::msg::TwistStamped cmd;
        cmd.header.stamp = this->now();
        cmd.header.frame_id = "base_link";
        cmd.twist.linear.x = 0.0;
        cmd.twist.angular.z = 0.0;
        vel_pub_->publish(cmd);
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrapezoidVelocityNode>());
    rclcpp::shutdown();
    return 0;
}
