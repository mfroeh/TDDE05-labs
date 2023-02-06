#include "nav2_msgs/msg/speed_limit.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <QPoint>
#include <functional>
#include <future>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/quaternion__struct.hpp>
#include <irobot_create_msgs/action/detail/navigate_to_position__struct.hpp>
#include <memory>
#include <nav2_msgs/msg/detail/speed_limit__struct.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "irobot_create_msgs/action/navigate_to_position.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

class DriveToExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  using DriveTo = irobot_create_msgs::action::NavigateToPosition;
  using GoalHandleDriveTo = rclcpp_action::ClientGoalHandle<DriveTo>;

  DriveToExecutor(const TstML::TSTNode *_node,
                  TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node = rclcpp::Node::make_shared(("drive_to_node") +
                                       std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });
    publisher_ =
        m_node->create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", 10);

    m_client_ptr = rclcpp_action::create_client<DriveTo>(m_node, "drive_to");
  }
  ~DriveToExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  TstML::Executor::ExecutionStatus start() override {
    using namespace std::placeholders;

    auto goal_msg = DriveTo::Goal();

    QVariant p =
        node()->getParameter(TstML::TSTNode::ParameterType::Specific, "p");

    auto map{p.toMap()};
    double x{map["x"].toDouble()};
    double y{map["y"].toDouble()};
    double z{map["z"].toDouble()};
    geometry_msgs::msg::Point point{};
    point.x = x;
    point.y = y;
    point.z = z;
    goal_msg.goal_pose.pose.position = point;

    if (node()->hasParameter(TstML::TSTNode::ParameterType::Specific,
                             "heading")) {
      double yaw =
          node()
              ->getParameter(TstML::TSTNode::ParameterType::Specific, "heading")
              .toDouble();
      geometry_msgs::msg::Quaternion orientation{};
      orientation.x = 0;
      orientation.y = 0;
      orientation.z = std::sin(yaw / 2);
      orientation.w = std::cos(yaw / 2);
      goal_msg.goal_pose.pose.orientation = orientation;
    }

    if (node()->hasParameter(TstML::TSTNode::ParameterType::Specific,
                             "maximum-speed")) {
      double max_speed =
          node()
              ->getParameter(TstML::TSTNode::ParameterType::Specific,
                             "maximum-speed")
              .toDouble();
      nav2_msgs::msg::SpeedLimit speed_limit{};
      speed_limit.speed_limit = max_speed;
      publisher_->publish(speed_limit);
    }

    auto send_goal_options = rclcpp_action::Client<DriveTo>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&DriveToExecutor::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&DriveToExecutor::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&DriveToExecutor::result_callback, this, _1);
    m_client_ptr->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(m_node->get_logger(), "%f, %f, %f", x, y, z);

    return TstML::Executor::ExecutionStatus::Started();
  }

  void goal_response_callback(
          std::shared_future<GoalHandleDriveTo::SharedPtr> future) {
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
      feedback_callback(GoalHandleDriveTo::SharedPtr goal_handle,
              const std::shared_ptr<const DriveTo::Feedback> feedback) {}

  void result_callback(const GoalHandleDriveTo::WrappedResult &result) {
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
  rclcpp_action::Client<DriveTo>::SharedPtr m_client_ptr;
  GoalHandleDriveTo::SharedPtr m_goal_handle;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
};
