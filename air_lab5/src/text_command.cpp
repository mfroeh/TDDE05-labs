#include <functional>
#include <memory>
#include <sstream>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include "air_lab_interfaces/msg/goal.hpp"

using namespace std::chrono_literals;

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class MyNode : public rclcpp::Node {
  public:
    MyNode() : Node{"mynode"} {
      publisher_ = create_publisher<air_lab_interfaces::msg::>("goals_requests", 10);
      subscription_ = create_subscription<std_msgs::msg::String>("text_command", 10, std::bind(&MyNode::text_command_callback, this, std::placeholders::_1));
    }

  private:
    rclcpp::Publisher<air_lab_interfaces::msg::Goals>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    void text_command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
