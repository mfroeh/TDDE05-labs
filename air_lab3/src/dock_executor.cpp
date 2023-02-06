#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "irobot_create_msgs/action/dock_servo.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

// Ugly hack to get a new name for each node
std::string gen_name(const std::string &_name) {
  static int counter = 0;
  return _name + std::to_string(++counter);
}

class DockExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  using Dock = irobot_create_msgs::action::DockServo;
  using GoalHandleDock = rclcpp_action::ClientGoalHandle<Dock>;

  DockExecutor(const TstML::TSTNode *_node,
               TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node = rclcpp::Node::make_shared(("dock") + std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });

    m_client_ptr = rclcpp_action::create_client<Dock>(m_node, "dock");
  }
  ~DockExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  TstML::Executor::ExecutionStatus start() override {
    using namespace std::placeholders;

    auto goal_msg = Dock::Goal();

    auto send_goal_options = rclcpp_action::Client<Dock>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DockExecutor::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&DockExecutor::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&DockExecutor::result_callback, this, _1);
    m_client_ptr->async_send_goal(goal_msg, send_goal_options);

    return TstML::Executor::ExecutionStatus::Started();
  }

  void
  goal_response_callback(std::shared_future<GoalHandleDock::SharedPtr> future) {
    m_goal_handle = future.get();
    if (!m_goal_handle) {
      executionFinished(TstML::Executor::ExecutionStatus::Aborted());
      RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(m_node->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(GoalHandleDock::SharedPtr goal_handle,
                         const std::shared_ptr<const Dock::Feedback> feedback) {
  }

  void result_callback(const GoalHandleDock::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(m_node->get_logger(), "Goal was succeeded");
      executionFinished(TstML::Executor::ExecutionStatus::Finished());
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was aborted");
      executionFinished(TstML::Executor::ExecutionStatus::Aborted());
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(m_node->get_logger(), "Goal was canceled");
      executionFinished(TstML::Executor::ExecutionStatus::Aborted());
      return;
    default:
      RCLCPP_ERROR(m_node->get_logger(), "Unknown result code");
      executionFinished(TstML::Executor::ExecutionStatus::Aborted());
      return;
    }
  }

  TstML::Executor::ExecutionStatus pause() override {
    return TstML::Executor::ExecutionStatus::Running();
  }
  TstML::Executor::ExecutionStatus resume() override {
    return TstML::Executor::ExecutionStatus::Running();
  }
  TstML::Executor::ExecutionStatus stop() override {
    m_client_ptr->async_cancel_goal(m_goal_handle);
    return TstML::Executor::ExecutionStatus::Finished();
  }
  TstML::Executor::ExecutionStatus abort() override {
    m_client_ptr->async_cancel_goal(m_goal_handle);
    return TstML::Executor::ExecutionStatus::Aborted();
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::executors::MultiThreadedExecutor m_executor;
  std::thread m_executor_thread;
  rclcpp_action::Client<Dock>::SharedPtr m_client_ptr;
  GoalHandleDock::SharedPtr m_goal_handle;
};
