#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <fstream>
#include <sstream>
#include <cmath>

class SyncTB3 : public rclcpp::Node
{
public:
  SyncTB3() : Node("sync_tb3")
  {
    RCLCPP_INFO(this->get_logger(), "Création du clone TurtleBot3...");

    // === Spawn du clone ===
    /*client_spawn_ = this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    while (!client_spawn_->wait_for_service(std::chrono::seconds(2)))
      RCLCPP_WARN(this->get_logger(), "En attente du service /spawn_entity...");

    std::ifstream ifs("/opt/ros/humble/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf");
    std::stringstream buffer;
    buffer << ifs.rdbuf();

    auto req = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    req->name = "tb3_2";
    req->xml = buffer.str();
    req->robot_namespace = "tb3_2";
    req->initial_pose.position.x = 1.0;
    req->initial_pose.position.y = 1.0;

    auto result = client_spawn_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS)
      RCLCPP_INFO(this->get_logger(), "✅ Clone créé !");
    else
      RCLCPP_ERROR(this->get_logger(), "❌ Erreur de création du clone.");*/

    // === Publishers ===
    pub_master_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    pub_clone_  = this->create_publisher<geometry_msgs::msg::Twist>("/tb3_2/cmd_vel", 10);

    // === Souscriptions ===
    sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10,
      std::bind(&SyncTB3::goal_callback, this, std::placeholders::_1));

    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&SyncTB3::odom_callback, this, std::placeholders::_1));

    // 🔹 LIDAR du maître
    sub_scan_master_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&SyncTB3::scan_master_callback, this, std::placeholders::_1));

    // 🔹 LIDAR du clone
    sub_scan_clone_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/tb3_2/scan", 10,
      std::bind(&SyncTB3::scan_clone_callback, this, std::placeholders::_1));

    // Timer pour le mouvement
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SyncTB3::move_toward_goal, this));
  }

private:
  // === Données internes ===
  double current_x_ = 0.0, current_y_ = 0.0, current_theta_ = 0.0;
  double goal_x_ = 0.0, goal_y_ = 0.0;
  bool goal_received_ = false;
  bool obstacle_master_ = false;
  bool obstacle_clone_ = false;

  // --- callbacks ---
  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_received_ = true;
    RCLCPP_INFO(this->get_logger(), "🎯 Nouvel objectif reçu: (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;

    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    current_theta_ = std::atan2(2.0 * (qw*qz + qx*qy),
                                1.0 - 2.0 * (qy*qy + qz*qz));
  }

  // === Détection LIDAR sur 360° ===
  bool detect_obstacle(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->ranges.size(); i++)
    {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r < 0.2)  // distance de sécurité = 0.2 m
        return true;
    }
    return false;
  }

  void scan_master_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    bool previous = obstacle_master_;
    obstacle_master_ = detect_obstacle(msg);
    if (obstacle_master_ && !previous)
      RCLCPP_WARN(this->get_logger(), "🚧 Obstacle détecté autour du maître !");
  }

  void scan_clone_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    bool previous = obstacle_clone_;
    obstacle_clone_ = detect_obstacle(msg);
    if (obstacle_clone_ && !previous)
      RCLCPP_WARN(this->get_logger(), "🚧 Obstacle détecté autour du clone !");
  }

  // === Mouvement synchronisé ===
  void move_toward_goal()
  {
    geometry_msgs::msg::Twist cmd;

    // ⚠️ Si l'un des deux détecte un obstacle → arrêt immédiat
    if (obstacle_master_ || obstacle_clone_) {
      pub_master_->publish(cmd);
      pub_clone_->publish(cmd);
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "⚠️ Obstacle détecté par l'un des robots : arrêt total !");
      return;
    }

    if (!goal_received_)
      return;

    double dx = goal_x_ - current_x_;
    double dy = goal_y_ - current_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double angle_to_goal = std::atan2(dy, dx);
    double angle_error = angle_to_goal - current_theta_;

    while (angle_error > M_PI) angle_error -= 2 * M_PI;
    while (angle_error < -M_PI) angle_error += 2 * M_PI;

    if (distance > 0.05) {
      cmd.linear.x = 0.15;
      cmd.angular.z = 1.0 * angle_error;
    } else {
      goal_received_ = false;
      RCLCPP_INFO(this->get_logger(), "✅ Objectif atteint !");
    }

    pub_master_->publish(cmd);
    pub_clone_->publish(cmd);
  }

  // === ROS members ===
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client_spawn_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_master_, pub_clone_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_master_, sub_scan_clone_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncTB3>());
  rclcpp::shutdown();
  return 0;
}
