#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <vector>

#include <ros2_kdb_msgs/srv/detail/query_database__struct.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "air_lab_interfaces/msg/goal.hpp"
#include "air_lab_interfaces/msg/goals.hpp"
#include <algorithm>

using namespace std::chrono_literals;

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

using air_lab_interfaces::msg::Goal;
using air_lab_interfaces::msg::Goals;

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node{"decision"} {
    subscription_ = create_subscription<Goals>("goal_requests", 10, std::bind(&DecisionNode::decision_callback, this, std::placeholders::_1));
  }

private:
  rclcpp::Subscription<Goals>::SharedPtr subscription_;

  void decision_callback(const Goals::SharedPtr msg) {
    std::vector<Goal> goals{msg->goals};

    RCLCPP_INFO(this->get_logger(), "Starting query\n");
    auto request{
        std::make_shared<ros2_kdb_msgs::srv::QueryDatabase::Request>()};
    request->graphname = "semanticobject";

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl;
    os << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
       << std::endl;
    os << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class ;" << std::endl;
    os << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;

    request->query = os.str();
    request->format = "json";

  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionNode>());
  rclcpp::shutdown();
  return 0;
}
