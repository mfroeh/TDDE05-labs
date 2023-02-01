#include "rclcpp/rclcpp.hpp"
#include "air_lab_interfaces/srv/execute_tst.hpp"

#include <memory>
#include <string>

void execute(const std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Request> request,
        std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Response>      response)
{
    std::string name{request->tst_file};
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Name: %s",
            name.c_str());
    response->success = true;
    response->error_message = "";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("execute_tst_server");

    rclcpp::Service<air_lab_interfaces::srv::ExecuteTst>::SharedPtr service =
        node->create_service<air_lab_interfaces::srv::ExecuteTst>("execute_tst", &execute);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to execute TSTs.");

    rclcpp::spin(node);
    rclcpp::shutdown();
}
