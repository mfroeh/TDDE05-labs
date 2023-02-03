#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

double euclidean(Point const& p1, Point const& p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

class MyNode : public rclcpp::Node {
  public:
    MyNode() : Node{"mynode"}, first_position{}, count_{} {
      declare_parameter("linear", 0.1);
      declare_parameter("angular", 0.05);
      declare_parameter("distance", 1.0);

      publisher_ = create_publisher<Twist>("cmd_vel", 10);
      subscription_ = create_subscription<Odometry>("odom", 10, std::bind(&MyNode::odom_callback, this, std::placeholders::_1));
      timer_ = create_wall_timer(500ms, std::bind(&MyNode::timer_callback, this));
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Twist>::SharedPtr publisher_;
    rclcpp::Subscription<Odometry>::SharedPtr subscription_;
    Point first_position;
    size_t count_;
    double distance;

    void timer_callback() {
      Twist message{};
      double linear{get_parameter("linear").get_parameter_value().get<double>()};
      double angular{get_parameter("angular").get_parameter_value().get<double>()};
      distance = get_parameter("distance").get_parameter_value().get<double>();

      // Didn't work
      /* message.linear = Vector3{0.1, 0.0, 0.0}; */
      message.linear = Vector3{};
      message.linear.x = linear;
      message.angular = Vector3{};
      message.angular.x = angular;
      RCLCPP_INFO(this->get_logger(), "Publishing:\n%s", this->twist_to_string(message).c_str());
      publisher_->publish(message);
    }

    void odom_callback(const Odometry::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "I heard:\n%s", point_to_string(msg->pose.pose.position).c_str());
      check_exit(msg);
    }

    void check_exit(Odometry::SharedPtr const& msg) {
      Point pos = msg->pose.pose.position;
      if (count_++ == 0) {
	first_position = pos;
      } else if (euclidean(pos, first_position) > distance) {
	RCLCPP_INFO(this->get_logger(), "Moved enough, shutting down.");
	rclcpp::shutdown();
      }
    }

    std::string twist_to_string(Twist const& msg) {
      std::ostringstream oss{};
      oss << "{\n";
      oss << "  linear = {" << msg.linear.x << ", " << msg.linear.y << ", " << msg.linear.z << "}\n";
      oss << "  angular = {" << msg.angular.x << ", " << msg.angular.y << ", " << msg.angular.z << "}\n";
      oss << "}\n";
      return oss.str();
    }

    std::string point_to_string(Point const& position) {
      std::ostringstream oss{};
      oss << "{ " << position.x << ", " << position.y << ", " << position.z << " }\n";
      return oss.str();
    }

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
  return 0;
}
