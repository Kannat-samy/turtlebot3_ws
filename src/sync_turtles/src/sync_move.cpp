#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/srv/spawn.hpp"

class SyncMove : public rclcpp::Node
{
public:
  SyncMove() : Node("sync_move")
  {
    client_spawn_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

    // Attente du service /spawn
    while (!client_spawn_->wait_for_service(std::chrono::seconds(1)))
      RCLCPP_INFO(this->get_logger(), "En attente du service /spawn...");

    // Prépare la requête
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->x = 2.0;
    request->y = 2.0;
    request->theta = 0.0;
    request->name = "turtle2";

    // Appel synchrone pour attendre la création réelle
    auto result = client_spawn_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result)
        == rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(this->get_logger(), "Turtle2 créée avec succès !");
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Échec du spawn de turtle2 !");
      return;
    }

    // Abonnement et publication
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/turtle1/cmd_vel", 10,
      std::bind(&SyncMove::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
  }

private:
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    pub_->publish(*msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_spawn_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SyncMove>());
  rclcpp::shutdown();
  return 0;
}

