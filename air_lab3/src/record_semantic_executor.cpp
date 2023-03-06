#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <TstML/Executor/ExecutionStatus.h>
#include <functional>
#include <future>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/future_return_code.hpp>
#include <rclcpp/logging.hpp>
#include <ros2_kdb_msgs/srv/detail/query_database__struct.hpp>
#include <sstream>
#include <string>
#include <thread>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "air_simple_sim_msgs/msg/semantic_observation.hpp"
#include "ros2_kdb_msgs/srv/insert_triples.hpp"
#include "ros2_kdb_msgs/srv/query_database.hpp"

#include <TstML/Executor/AbstractNodeExecutor.h>
#include <TstML/TSTNode.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using air_simple_sim_msgs::msg::SemanticObservation;
using QueryServiceT = ros2_kdb_msgs::srv::QueryDatabase;
using InsertServiceT = ros2_kdb_msgs::srv::InsertTriples;
using geometry_msgs::msg::PointStamped;

class RecordSemanticExecutor : public TstML::Executor::AbstractNodeExecutor {
public:
  RecordSemanticExecutor(const TstML::TSTNode *_node,
                         TstML::Executor::AbstractExecutionContext *_context)
      : TstML::Executor::AbstractNodeExecutor(_node, _context) {
    static int counter{};
    m_node = rclcpp::Node::make_shared("record_semantic_node" +
                                       std::to_string(++counter));
    m_executor.add_node(m_node);
    m_executor_thread = std::thread([this]() { m_executor.spin(); });

    // The tf_buffer and tf_listener needs to be kept alive and should be
    // created in a constructor
    tf_buffer = std::make_unique<tf2_ros::Buffer>(m_node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
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

    query_client =
        m_node->create_client<QueryServiceT>("/kdb_server/sparql_query");
    insert_client =
        m_node->create_client<InsertServiceT>("/kdb_server/insert_triples");
    query_client->wait_for_service();
    insert_client->wait_for_service();
    return TstML::Executor::ExecutionStatus::Started();
  }

  ~RecordSemanticExecutor() {
    m_executor.cancel();
    m_executor_thread.join();
  }

  void semantic_callback(const SemanticObservation::SharedPtr msg) {
    RCLCPP_INFO(this->m_node->get_logger(), "I heard:\n%f", msg->point.point.x);
    RCLCPP_INFO(this->m_node->get_logger(),
                "graphname: %s, uuid: %s, klass: %s\n", graph_name.c_str(),
                msg->uuid.c_str(), msg->klass.c_str());
    auto request{std::make_shared<QueryServiceT::Request>()};
    request->graphname = graph_name;

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl
       << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
       << std::endl
       << "SELECT ?x ?y WHERE { <" << msg->uuid.c_str() << "> a <"
       << msg->klass.c_str() << "> ;" << std::endl;
    os << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;
    request->query = os.str();

    RCLCPP_INFO(this->m_node->get_logger(), "query: %s\n",
                request->query.c_str());
    auto result = query_client->async_send_request(
        request,
        [this, msg](rclcpp::Client<QueryServiceT>::SharedFuture future) {
          RCLCPP_INFO(this->m_node->get_logger(), "%d\n",
                      future.get()->success);

          QJsonDocument doc = QJsonDocument::fromJson(
              QByteArray::fromStdString(future.get()->result));

          RCLCPP_INFO(this->m_node->get_logger(), "%s\n",
                      doc.toJson().toStdString().c_str());
          if (doc.object()["results"].toObject()["bindings"].toArray().size() ==
              0) {
            insert(msg);
          }
        });
  }

  void insert(const SemanticObservation::SharedPtr msg) {
    PointStamped stamped{};
    stamped.header.frame_id = "turtlebot0/semantic_sensor";
    stamped.point.x = msg->point.point.x;
    stamped.point.y = msg->point.point.y;

    PointStamped p_after{};
    try {
      p_after = tf_buffer->transform(stamped, "map");
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(m_node->get_logger(), "Could not transform %s to %s: %s",
                  stamped.header.frame_id.c_str(), "map", ex.what());
    }

    std::ostringstream os{};
    // os << "graphname:" << graph_name << std::endl;
    // os << "format: 'ttl'" << std::endl;
    os << "@prefix gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl;
    os << "@prefix properties: <http://www.ida.liu.se/~TDDE05/properties>"
       << std::endl;
    os << "<" << msg->uuid.c_str() << "> a <" << msg->klass.c_str() << ">;"
       << std::endl;
    os << "properties:location [ gis:x " << msg->point.point.x << "; gis:y "
       << msg->point.point.y << " ] ." << std::endl;

    RCLCPP_INFO(m_node->get_logger(), "Inserting with statement: %s\n",
                os.str().c_str());

    auto request{std::make_shared<InsertServiceT::Request>()};
    request->graphname = graph_name;
    request->format = "ttl";
    request->content = os.str();
    auto result = insert_client->async_send_request(
        request, [this](rclcpp::Client<InsertServiceT>::SharedFuture future) {
          RCLCPP_INFO(this->m_node->get_logger(), "Inserted: %s\n",
                      future.get()->success ? "Success" : "Failure");
          RCLCPP_INFO(this->m_node->get_logger(), "Insert Future: %s\n",
                      future.get()->err_msgs.c_str());
        });
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
  rclcpp::Client<QueryServiceT>::SharedPtr query_client;
  rclcpp::Client<InsertServiceT>::SharedPtr insert_client;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};
