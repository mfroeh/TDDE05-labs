#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "air_simple_sim_msgs/msg/SemanticObservations.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

using air_simple_sim_msgs::msg::SemanticObservations;

class RecordSemanticExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  RecordSemanticExecutor(const TstML::TSTNode *_node,
                         TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node = rclcpp::Node::make_shared("_node" + std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });

    std::string topic{node()->getParameter(TsTML::TSTNode::ParameterType::Specific, "topic").to_string()};
    std::string graphname{node()->getParameter(TsTML::TSTNode::ParameterType::Specific, "graphname").to_string()};

    subscription = m_node.create_subscription<SemanticObservations>( topic, 10);
  }

  ~RecordSemanticExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
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
  rclcpp::Subscription<SemanticObservations>::SharedPtr subscription;
};
