#include "air_lab_interfaces/srv/execute_tst.hpp"
#include "rclcpp/rclcpp.hpp"
#include <QUrl>
#include <TstML/Executor/DefaultNodeExecutor/Concurrent.h>
#include <TstML/Executor/DefaultNodeExecutor/Sequence.h>
#include <TstML/Executor/ExecutionStatus.h>
#include <TstML/Executor/Executor.h>
#include <TstML/Executor/NodeExecutorRegistry.h>
#include <TstML/TSTNode.h>
#include <TstML/TSTNodeModelsRegistry.h>
#include <ament_index_cpp/get_package_prefix.hpp>

#include <memory>
#include <string>

using namespace std::placeholders;

class MyNode : public rclcpp::Node {

public:
  MyNode()
      : Node{"mynode"}, tst_registry{new TstML::TSTNodeModelsRegistry{}},
        tst_executor_registry{new TstML::Executor::NodeExecutorRegistry{}} {
    tst_registry->loadDirectory(
        QString::fromStdString(ament_index_cpp::get_package_prefix("air_tst") +
                               "/share/air_tst/configs"));
    service = create_service<air_lab_interfaces::srv::ExecuteTst>(
        "execute_tst", std::bind(&MyNode::execute, this, _1, _2));

    // Setup the executors for sequence and concurrent
    tst_executor_registry
        ->registerNodeExecutor<TstML::Executor::DefaultNodeExecutor::Sequence>(
            tst_registry->model("seq"));
    tst_executor_registry->registerNodeExecutor<
        TstML::Executor::DefaultNodeExecutor::Concurrent>(
        tst_registry->model("conc"));
  }

private:
  TstML::TSTNodeModelsRegistry *tst_registry;
  rclcpp::Service<air_lab_interfaces::srv::ExecuteTst>::SharedPtr service;
  TstML::Executor::NodeExecutorRegistry *tst_executor_registry;

  void execute(
      const std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Request>
          request,
      std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Response> response) {
    std::string name{request->tst_file};
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Name: %s", name.c_str());

    TstML::TSTNode *tst_node = TstML::TSTNode::load(
        QUrl::fromLocalFile(QString::fromStdString(name)), tst_registry);

    // Create an executor using the executors defined
    // in tst_executor_registry
    TstML::Executor::Executor *tst_executor =
        new TstML::Executor::Executor(tst_node, tst_executor_registry);

    // Start execution
    tst_executor->start();

    // Block until the execution has finished
    TstML::Executor::ExecutionStatus status = tst_executor->waitForFinished();

    // Display the result of execution
    response->success = status == TstML::Executor::ExecutionStatus::Finished();

    if (response->success) {
      RCLCPP_INFO(get_logger(), "Execution successful");
    } else {
      std::string message = status.message().toStdString();
      response->error_message = message;
      RCLCPP_INFO(get_logger(), "Execution successful '%s'", message.c_str());
    }

    delete tst_executor;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyNode>());
  rclcpp::shutdown();
}
