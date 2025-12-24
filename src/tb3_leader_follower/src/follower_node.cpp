#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

class FollowerNode : public rclcpp::Node
{
public:
  FollowerNode() : Node("tb3_follower")
  {
    // Paramètres
    this->declare_parameter<double>("target_distance", 0.10);   // 10 cm
    this->declare_parameter<double>("kp_dist", 1.0);
    this->declare_parameter<double>("kp_yaw", 2.0);
    this->declare_parameter<double>("max_lin_vel", 0.20);
    this->declare_parameter<double>("max_ang_vel", 1.0);
    this->declare_parameter<double>("deadband", 0.02);          // +/- 2 cm
    this->declare_parameter<double>("leader_stop_thresh", 0.02);

    this->get_parameter("target_distance", target_distance_);
    this->get_parameter("kp_dist", kp_dist_);
    this->get_parameter("kp_yaw", kp_yaw_);
    this->get_parameter("max_lin_vel", max_lin_vel_);
    this->get_parameter("max_ang_vel", max_ang_vel_);
    this->get_parameter("deadband", deadband_);
    this->get_parameter("leader_stop_thresh", leader_stop_thresh_);

    // ATTENTION : ces topics supposent que plus tard on aura deux robots namespacés
    // Leader : /tb3_1/odom
    // Follower : /tb3_2/odom  et /tb3_2/cmd_vel
    leader_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/tb3_1/odom", 10,
      std::bind(&FollowerNode::leaderOdomCallback, this, std::placeholders::_1));

    follower_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/tb3_2/odom", 10,
      std::bind(&FollowerNode::followerOdomCallback, this, std::placeholders::_1));

    follower_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/tb3_2/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&FollowerNode::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "FollowerNode initialisé.");
  }

private:
  void leaderOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    leader_odom_ = *msg;
    leader_odom_received_ = true;
  }

  void followerOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    follower_odom_ = *msg;
    follower_odom_received_ = true;
  }

  double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
  {
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  }

  double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  void controlLoop()
  {
    if (!leader_odom_received_ || !follower_odom_received_) {
      return;
    }

    double xL = leader_odom_.pose.pose.position.x;
    double yL = leader_odom_.pose.pose.position.y;
    double xF = follower_odom_.pose.pose.position.x;
    double yF = follower_odom_.pose.pose.position.y;

    double dx = xL - xF;
    double dy = yL - yF;
    double dist = std::sqrt(dx*dx + dy*dy);

    double follower_yaw = yawFromQuat(follower_odom_.pose.pose.orientation);
    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = normalizeAngle(desired_yaw - follower_yaw);

    double dist_error = dist - target_distance_;

    // Vitesses du leader
    double vL = leader_odom_.twist.twist.linear.x;
    double wL = leader_odom_.twist.twist.angular.z;

    geometry_msgs::msg::Twist cmd;

    // Cas où le leader est quasiment à l'arrêt ET on est à la bonne distance -> stop net
    if (std::fabs(vL) < leader_stop_thresh_ && std::fabs(dist_error) < deadband_) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
      follower_cmd_pub_->publish(cmd);
      return;
    }

    // Contrôle distance : ne pas pousser le leader
    if (dist < target_distance_) {
      if (dist_error < -deadband_) {
        cmd.linear.x = kp_dist_ * dist_error;
      } else {
        cmd.linear.x = 0.0;
      }
    } else {
      cmd.linear.x = kp_dist_ * dist_error;
    }

    // Contrôle orientation
    cmd.angular.z = kp_yaw_ * yaw_error;

    // Saturations
    if (cmd.linear.x > max_lin_vel_) cmd.linear.x = max_lin_vel_;
    if (cmd.linear.x < -max_lin_vel_) cmd.linear.x = -max_lin_vel_;
    if (cmd.angular.z > max_ang_vel_) cmd.angular.z = max_ang_vel_;
    if (cmd.angular.z < -max_ang_vel_) cmd.angular.z = -max_ang_vel_;

    follower_cmd_pub_->publish(cmd);
  }

  // Params
  double target_distance_;
  double kp_dist_;
  double kp_yaw_;
  double max_lin_vel_;
  double max_ang_vel_;
  double deadband_;
  double leader_stop_thresh_;

  // Etat
  nav_msgs::msg::Odometry leader_odom_;
  nav_msgs::msg::Odometry follower_odom_;
  bool leader_odom_received_ = false;
  bool follower_odom_received_ = false;

  // ROS
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr leader_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr follower_odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr follower_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

