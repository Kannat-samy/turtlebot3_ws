#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class FollowerNode : public rclcpp::Node
{
public:
  FollowerNode() : Node("follower_node")
  {
    // -------- PARAMETERS --------
    leader_odom_topic_   = declare_parameter("leader_odom", "/robot1/odom");
    follower_odom_topic_ = declare_parameter("follower_odom", "/robot2/odom");
    cmd_vel_topic_       = declare_parameter("cmd_vel", "/robot2/cmd_vel");

    target_distance_ = declare_parameter("target_distance", 0.5);
    kp_dist_ = declare_parameter("kp_dist", 0.6);
    ki_dist_ = declare_parameter("ki_dist", 0.05);
    kd_dist_ = declare_parameter("kd_dist", 0.15);
    kp_yaw_  = declare_parameter("kp_yaw", 2.0);

    max_lin_vel_ = declare_parameter("max_lin_vel", 0.22);
    max_ang_vel_ = declare_parameter("max_ang_vel", 1.0);
    max_lin_acc_ = declare_parameter("max_lin_acc", 0.5);

    deadband_ = declare_parameter("deadband", 0.02);
    yaw_deadband_ = declare_parameter("yaw_deadband", 0.05);

    leader_stop_thresh_ = declare_parameter("leader_stop_thresh", 0.02);
    leader_stop_hold_s_ = declare_parameter("leader_stop_hold_s", 0.3);

    i_max_ = declare_parameter("i_max", 0.25);

    // -------- ROS --------
    leader_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      leader_odom_topic_, 10, std::bind(&FollowerNode::leaderCb, this, _1));

    follower_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      follower_odom_topic_, 10, std::bind(&FollowerNode::followerCb, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FollowerNode::controlLoop, this));

    RCLCPP_INFO(get_logger(), "Follower node READY");
  }

private:
  // -------- CALLBACKS --------
  void leaderCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_ = *msg;
    leader_ok_ = true;

    double v = std::fabs(msg->twist.twist.linear.x);
    if (v < leader_stop_thresh_) {
      if (!leader_stopped_) {
        leader_stop_time_ = now();
        leader_stopped_ = true;
      }
    } else {
      leader_stopped_ = false;
    }
  }

  void followerCb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    follower_ = *msg;
    follower_ok_ = true;
  }

  // -------- UTILS --------
  double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
  {
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y*q.y + q.z*q.z));
  }

  double clamp(double v, double lo, double hi)
  {
    return std::max(lo, std::min(v, hi));
  }

  // -------- CONTROL --------
  void controlLoop()
  {
    if (!leader_ok_ || !follower_ok_) return;

    // HOLD strict si leader arrêté
    if (leader_stopped_) {
      if ((now() - leader_stop_time_).seconds() > leader_stop_hold_s_) {
        cmd_pub_->publish(geometry_msgs::msg::Twist());
        integral_ = 0.0;
        prev_v_ = 0.0;
        return;
      }
    }

    double xL = leader_.pose.pose.position.x;
    double yL = leader_.pose.pose.position.y;
    double xF = follower_.pose.pose.position.x;
    double yF = follower_.pose.pose.position.y;

    double dx = xL - xF;
    double dy = yL - yF;
    double dist = std::hypot(dx, dy);

    double e = dist - target_distance_;
    if (std::fabs(e) < deadband_) e = 0.0;

    integral_ = clamp(integral_ + e * 0.05, -i_max_, i_max_);
    double de = (e - prev_error_) / 0.05;
    prev_error_ = e;

    double v_leader = leader_.twist.twist.linear.x;
    double v_cmd = v_leader + kp_dist_ * e + ki_dist_ * integral_ + kd_dist_ * de;

    // Acceleration limit
    double dv = v_cmd - prev_v_;
    double max_dv = max_lin_acc_ * 0.05;
    dv = clamp(dv, -max_dv, max_dv);
    v_cmd = prev_v_ + dv;
    prev_v_ = v_cmd;

    v_cmd = clamp(v_cmd, -max_lin_vel_, max_lin_vel_);

    double yawF = yawFromQuat(follower_.pose.pose.orientation);
    double yaw_des = std::atan2(dy, dx);
    double yaw_err = yaw_des - yawF;
    if (std::fabs(yaw_err) < yaw_deadband_) yaw_err = 0.0;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v_cmd;
    cmd.angular.z = clamp(kp_yaw_ * yaw_err, -max_ang_vel_, max_ang_vel_);

    cmd_pub_->publish(cmd);
  }

  // -------- PARAMS --------
  std::string leader_odom_topic_, follower_odom_topic_, cmd_vel_topic_;
  double target_distance_;
  double kp_dist_, ki_dist_, kd_dist_, kp_yaw_;
  double max_lin_vel_, max_ang_vel_, max_lin_acc_;
  double deadband_, yaw_deadband_;
  double leader_stop_thresh_, leader_stop_hold_s_;
  double i_max_;

  // -------- STATE --------
  nav_msgs::msg::Odometry leader_, follower_;
  bool leader_ok_{false}, follower_ok_{false};
  bool leader_stopped_{false};

  rclcpp::Time leader_stop_time_;
  double prev_error_{0.0};
  double integral_{0.0};
  double prev_v_{0.0};

  // -------- ROS --------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerNode>());
  rclcpp::shutdown();
  return 0;
}
