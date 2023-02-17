#include <TstML/Executor/ExecutionStatus.h>
#include <functional>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <sstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "air_simple_sim_msgs/msg/semantic_observation.hpp"
#include "ros2_kdb_msgs/srv/query_database.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

using air_simple_sim_msgs::msg::SemanticObservation;
using ServiceT = ros2_kdb_msgs::srv::QueryDatabase;

class RecordSemanticExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  RecordSemanticExecutor(const TstML::TSTNode *_node,
                         TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node = rclcpp::Node::make_shared("_node" + std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });
  }

  TstML::Executor::ExecutionStatus start() override {
    std::string topic{
        node()
            ->getParameter(TstML::TSTNode::ParameterType::Specific, "topic")
            .toString()
            .toStdString()};
    graph_name =
        node()
            ->getParameter(TstML::TSTNode::ParameterType::Specific, "graphname")
            .toString()
            .toStdString();

    subscription = m_node->create_subscription<SemanticObservation>(
        topic, 10,
        std::bind(&RecordSemanticExecutor::semantic_callback, this,
                  std::placeholders::_1));

    client = m_node->create_client<ServiceT>("/kdb_server/sparql_query");
    return TstML::Executor::ExecutionStatus::Started();
  }

  ~RecordSemanticExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  void semantic_callback(const SemanticObservation::SharedPtr msg) {
    RCLCPP_INFO(this->m_node->get_logger(), "I heard:\n%f", msg->point.point.x);
    rclcpp::Client<ServiceT>::SharedRequest request;
    request->graphname = graph_name;
    request->query =
        "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>"
        "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
        "SELECT ?x ?y WHERE { <" +
        msg->uuid + "> a <" + msg->klass +
        "> ;"
        "properties:location [ gis:x ?x; gis:y ?y ] . }";
    auto response = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(m_node, response) ==
        rclcpp::FutureReturnCode ::SUCCESS) {
      RCLCPP_INFO(this->m_node->get_logger(), "Success");
    } else {
      RCLCPP_INFO(this->m_node->get_logger(), "Failure");
    }
  }

  TstML::Executor::ExecutionStatus pause() override {
    return TstML::Executor::ExecutionStatus::Running();
  }
  TstML::Executor::ExecutionStatus resume() override {
    return TstML::Executor::ExecutionStatus::Running();
  }
  TstML::Executor::ExecutionStatus stop() override {
    return TstML::Executor::ExecutionStatus::Finished();
  }
  TstML::Executor::ExecutionStatus abort() override {
    return TstML::Executor::ExecutionStatus::Aborted();
  }

private:
  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::executors::MultiThreadedExecutor m_executor;
  std::thread m_executor_thread;
  rclcpp::Subscription<SemanticObservation>::SharedPtr subscription;
  std::string graph_name;
  rclcpp::Client<ServiceT>::SharedPtr client;
};
