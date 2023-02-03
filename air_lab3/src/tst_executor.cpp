#include "rclcpp/rclcpp.hpp"
#include <TstML/TSTNodeModelsRegistry.h>
#include <TstML/TSTNode.h>
#include "air_lab_interfaces/srv/execute_tst.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <QUrl>

#include <memory>
#include <string>

class MyNode:public rclcpp::Node{

    public:
        MyNode(): Node{"mynode"}, tst_registry{new TstML::TSTNodeModelsRegistry{}} {
            tst_registry->loadDirectory(
                    QString::fromStdString(ament_index_cpp::get_package_prefix("air_tst")
                        + "/share/air_tst/configs"));

        }
        void execute(const std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Request> request,
                std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Response>      response)
        {
            std::string name{request->tst_file};
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Name: %s",
                    name.c_str());
            TstML::TSTNode* tst_node = TstML::TSTNode::load(
                        QUrl::fromLocalFile(QString::fromStdString(name)),
                                tst_registry);
            response->success = true;
            response->error_message = "";
        }

    private:
        TstML::TSTNodeModelsRegistry* tst_registry;
};


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
