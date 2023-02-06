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
#include <queue>
#include <sstream>
#include <string>
#include <thread>

#include "irobot_create_msgs/action/navigate_to_position.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>
#include <vector>

class ExploreExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  using Explore = irobot_create_msgs::action::NavigateToPosition;
  using GoalHandleExplore = rclcpp_action::ClientGoalHandle<Explore>;

  ExploreExecutor(const TstML::TSTNode *_node,
                  TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node =
        rclcpp::Node::make_shared(("explore_node") + std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });
    m_client_ptr = rclcpp_action::create_client<Explore>(m_node, "explore");

    double radius{
        node()
            ->getParameter(TstML::TSTNode::ParameterType::Specific, "radius")
            .toDouble()};
    double a{node()
                 ->getParameter(TstML::TSTNode::ParameterType::Specific, "a")
                 .toDouble()};
    double b{node()
                 ->getParameter(TstML::TSTNode::ParameterType::Specific, "b")
                 .toDouble()};

    double theta{};
    double cur_r{a + b * theta};
    while (cur_r < radius) {
      theta += 3.14 / 4;
      cur_r = a + b * theta;

      geometry_msgs::msg::Point p{};
      p.x = std::cos(theta) * cur_r;
      p.y = std::sin(theta) * cur_r;
      waypoints.push(p);
    }
  }

  ~ExploreExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  TstML::Executor::ExecutionStatus start() override {
    using namespace std::placeholders;

    auto goal_msg = Explore::Goal();

    auto front{waypoints.front()};
    waypoints.pop();
    goal_msg.goal_pose.pose.position = front;

    auto send_goal_options = rclcpp_action::Client<Explore>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&ExploreExecutor::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&ExploreExecutor::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&ExploreExecutor::result_callback, this, _1);
    m_client_ptr->async_send_goal(goal_msg, send_goal_options);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Send goal! ");
    return TstML::Executor::ExecutionStatus::Started();
  }

  void goal_response_callback(
      std::shared_future<GoalHandleExplore::SharedPtr> future) {
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
  feedback_callback(GoalHandleExplore::SharedPtr goal_handle,
                    const std::shared_ptr<const Explore::Feedback> feedback) {}

  void result_callback(const GoalHandleExplore::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(m_node->get_logger(), "Goal was succeeded");
      start();
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
  rclcpp_action::Client<Explore>::SharedPtr m_client_ptr;
  GoalHandleExplore::SharedPtr m_goal_handle;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
  std::queue<geometry_msgs::msg::Point> waypoints;
};
