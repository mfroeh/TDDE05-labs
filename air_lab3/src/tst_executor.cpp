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
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rmw/qos_profiles.h>
#include <string>

#include "std_srvs/srv/empty.hpp"
#include "undock_executor.cpp"
#include "dock_executor.cpp"
#include "drive_to_executor.cpp"
#include "explore_executor.cpp"

using namespace std::placeholders;
using std_srvs::srv::Empty;

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

    rclcpp::CallbackGroup::SharedPtr group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    abort_service = create_service<Empty>("abort", std::bind(&MyNode::abort, this, _1, _2), rmw_qos_profile_services_default, group);
    stop_service = create_service<Empty>("stop", std::bind(&MyNode::stop, this, _1, _2), rmw_qos_profile_services_default, group);
    pause_service = create_service<Empty>("pause", std::bind(&MyNode::pause, this, _1, _2), rmw_qos_profile_services_default, group);
    resume_service = create_service<Empty>("resume", std::bind(&MyNode::resume, this, _1, _2), rmw_qos_profile_services_default, group);

    // Setup the executors for sequence and concurrent
    tst_executor_registry
        ->registerNodeExecutor<TstML::Executor::DefaultNodeExecutor::Sequence>(
            tst_registry->model("seq"));
    tst_executor_registry->registerNodeExecutor< TstML::Executor::DefaultNodeExecutor::Concurrent>( tst_registry->model("conc"));
    tst_executor_registry->registerNodeExecutor<UndockExecutor>( tst_registry->model("undock"));
    tst_executor_registry->registerNodeExecutor<DockExecutor>( tst_registry->model("dock"));
    tst_executor_registry->registerNodeExecutor<DriveToExecutor>( tst_registry->model("drive-to"));
    tst_executor_registry->registerNodeExecutor<ExploreExecutor>( tst_registry->model("explore"));
  }

private:
  TstML::TSTNodeModelsRegistry *tst_registry;
  rclcpp::Service<air_lab_interfaces::srv::ExecuteTst>::SharedPtr service;
  rclcpp::Service<Empty>::SharedPtr abort_service;
  rclcpp::Service<Empty>::SharedPtr stop_service;
  rclcpp::Service<Empty>::SharedPtr pause_service;
  rclcpp::Service<Empty>::SharedPtr resume_service;
  TstML::Executor::NodeExecutorRegistry *tst_executor_registry;
  TstML::Executor::Executor* tst_executor;

  void execute(
      const std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Request>
          request,
      std::shared_ptr<air_lab_interfaces::srv::ExecuteTst::Response> response) {
    std::string name{request->tst_file};
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Name: %s", name.c_str());

    TstML::TSTNode *tst_node = TstML::TSTNode::load(
        QUrl::fromLocalFile(QString::fromStdString(name)), tst_registry);

    // Create an executor using the executors defined in tst_executor_registry
    tst_executor = new TstML::Executor::Executor(tst_node, tst_executor_registry);

    // Start execution
    tst_executor->start();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Before waitForFinished", name.c_str());
    // Block until the execution has finished
    TstML::Executor::ExecutionStatus status = tst_executor->waitForFinished(10000);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "After waitForFinished", name.c_str());

    // Display the result of execution
    response->success = status == TstML::Executor::ExecutionStatus::Finished();

    if (response->success) {
      RCLCPP_INFO(get_logger(), "Execution successful");
    } else {
      std::string message = status.message().toStdString();
      response->error_message = message;
      RCLCPP_INFO(get_logger(), "Execution error '%s'", message.c_str());
    }

    delete tst_executor;
  }

  void abort(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response) {
      if (tst_executor != NULL) {
          tst_executor->abort();
      }
  }

  void stop(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response) {
      if (tst_executor != NULL) {
          tst_executor->stop();
      }
  }

  void pause(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response) {
      if (tst_executor != NULL) {
          tst_executor->pause();
      }
  }

  void resume(const std::shared_ptr<Empty::Request> request,
             std::shared_ptr<Empty::Response> response) {
      if (tst_executor != NULL) {
          tst_executor->resume();
      }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<MyNode>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
