#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>
#include <string>

#include "undock_executor.cpp"

using namespace std::placeholders;
using std_srvs::srv::Empty;

class StatusServer : public rclcpp::Node {
public:
  StatusServer() : Node{"status_server"} {
    abort_service = create_service<Empty>(
        "abort", std::bind(&StatusServer::abort, this, _1, _2));
  }

private:
  rclcpp::Service<Empty>::SharedPtr abort_service;

  void abort(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response) {

  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
