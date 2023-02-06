#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "irobot_create_msgs/action/undock.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

class UndockExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  using Undock = irobot_create_msgs::action::Undock;
  using GoalHandleUndock = rclcpp_action::ClientGoalHandle<Undock>;

  UndockExecutor(const TstML::TSTNode *_node,
                 TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node =
        rclcpp::Node::make_shared("undock_node" + std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });

    m_client_ptr = rclcpp_action::create_client<Undock>(m_node, "undock");
  }
  ~UndockExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  TstML::Executor::ExecutionStatus start() override {
    using namespace std::placeholders;

    auto goal_msg = Undock::Goal();

    auto send_goal_options = rclcpp_action::Client<Undock>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&UndockExecutor::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&UndockExecutor::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&UndockExecutor::result_callback, this, _1);
    m_client_ptr->async_send_goal(goal_msg, send_goal_options);

    return TstML::Executor::ExecutionStatus::Started();
  }

  void goal_response_callback(
      std::shared_future<GoalHandleUndock::SharedPtr> future) {
    m_goal_handle = future.get();
    if (!m_goal_handle) {
      executionFinished(TstML::Executor::ExecutionStatus::Aborted());
      RCLCPP_ERROR(m_node->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(m_node->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void
  feedback_callback(GoalHandleUndock::SharedPtr goal_handle,
                    const std::shared_ptr<const Undock::Feedback> feedback) {}

  void result_callback(const GoalHandleUndock::WrappedResult &result) {
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
  rclcpp_action::Client<Undock>::SharedPtr m_client_ptr;
  GoalHandleUndock::SharedPtr m_goal_handle;
};
