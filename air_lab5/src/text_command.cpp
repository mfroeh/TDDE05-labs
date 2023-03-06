#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "air_lab_interfaces/msg/goal.hpp"
#include <algorithm>

using namespace std::chrono_literals;

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

class MyNode : public rclcpp::Node {
public:
  MyNode() : Node{"mynode"} {
    publisher_ =
        create_publisher<air_lab_interfaces::msg::Goals>("goals_requests", 10);
    subscription_ = create_subscription<std_msgs::msg::String>(
        "text_command", 10,
        std::bind(&MyNode::text_command_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Publisher<air_lab_interfaces::msg::Goals>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  void text_command_callback(const std_msgs::msg::String::SharedPtr msg) {
    std::string cmd{};
    std::transform(std::begin(msg->data), std::end(msg->data),
                   std::back_inserter(cmd),
                   [](auto &&c) { return std::tolower(c); });

    std::istringstream iss{cmd};

    std::string type{};
    iss >> type;

    // Explore!
    if (type[type.size() - 1] == '!') {
      type.pop_back();
    }

    std::string word{};
    std::string destination{};
    std::string object{};

    if (iss >> word) {
      // Filter out objects
      if (word == "coffee") {
        object = word;
        object[0] = std::toupper(object[0]);
      }
      // Everything else must be destinations
      else {
        destination = word;
        destination[0] = std::toupper(destination[0]);
      }
    }

    if (iss >> word) {
      destination = word;
      destination[0] = std::toupper(destination[0]);
    }

    if (object != "" && destination == "") {
      destination = "User";
    }

    air_lab_interfaces::msg::Goal g{};
    g.type = type;
    g.object = object;
    g.destination = destination;

    Goals goals{};
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
