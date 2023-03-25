#include "air_lab_interfaces/srv/execute_tst.hpp"

#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <vector>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <ros2_kdb_msgs/srv/detail/query_database__struct.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace rclcpp;

using geometry_msgs::msg::Point;
using geometry_msgs::msg::PointStamped;
using ros2_kdb_msgs::srv::QueryDatabase;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class HumanDriver : public Node {
public:
  HumanDriver() : Node{"HumanDriver"} { query(); }

private:
  Client<QueryDatabase>::SharedPtr client;

  std::string graphname{"semanticobject"};

  struct Object {
    std::string uuid;
    std::string klass;
    double x;
    double y;
  };

  void generate_tst(std::vector<Object> &&objects) {
    QJsonArray children;

    for (auto &&human : objects) {
      QJsonArray drive_to_children;
      QJsonObject params;

      // Create the Point object for the 'p' parameter
      QJsonObject p;
      p.insert("rostype", "Point");
      p.insert("x", human.x);
      p.insert("y", human.y);
      p.insert("z", 0);

      // Add the 'p' parameter to the params object
      params.insert("p", p);

      // Set the 'use-motion-planner' parameter
      params.insert("use-motion-planner", true);

      // Create the drive_to node
      QJsonObject drive_to_node;
      drive_to_node.insert("name", "drive-to");
      drive_to_node.insert("common_params", QJsonObject());
      drive_to_node.insert("params", params);
      drive_to_node.insert("children", drive_to_children);

      // Add the drive_to node to the children array
      children.append(drive_to_node);

    // Create the seq node
    QJsonObject seq_node;
    seq_node.insert("name", "seq");
    seq_node.insert("common_params", QJsonObject());
    seq_node.insert("params", QJsonObject());
    seq_node.insert("children", children);

    // Create the JSON document
    QJsonDocument doc(seq_node);

    // Write the document to a file named 'tst.json'
    QFile file("drive_to.json");
    if (file.open(QIODevice::WriteOnly)) {
      file.write(doc.toJson());
      file.close();
    }


  rclcpp::Client<air_lab_interfaces::srv::ExecuteTst>::SharedPtr client = create_client<air_lab_interfaces::srv::ExecuteTst>(
        "execute_tst");
    }
    RCLCPP_INFO(this->get_logger(), "Waiting on service");
    client->wait_for_service();
    RCLCPP_INFO(this->get_logger(), "Service online\n");

    auto request{std::make_shared<air_lab_interfaces::srv::ExecuteTst::Request>()};
    request->tst_file = "drive_to.json";
    auto response = client->async_send_request(request);
  }

public:
  void query() {
    RCLCPP_INFO(this->get_logger(), "Starting query\n");
    auto request{
        std::make_shared<ros2_kdb_msgs::srv::QueryDatabase::Request>()};
    request->graphname = graphname;

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl;
    os << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
       << std::endl;
    os << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class ;" << std::endl;
    os << "properties:location [ gis:x ?x; gis:y ?y ] . }" << std::endl;

    request->query = os.str();
    request->format = "json";

    client = create_client<QueryDatabase>("/kdb_server/sparql_query");
    auto response = client->async_send_request(
        request, [this](Client<QueryDatabase>::SharedFuture future) {
          QJsonDocument doc{QJsonDocument::fromJson(
              QByteArray::fromStdString(future.get()->result))};

          RCLCPP_INFO(this->get_logger(), "Received result\n");

          RCLCPP_INFO(this->get_logger(), "Object: %s\n",
                      future.get()->result.c_str());
          if (!future.get()->success) {
            RCLCPP_ERROR(get_logger(),
                         "Query: /kdb_server/sparql_query wasn't successful!");
            return;
          }

          RCLCPP_INFO(this->get_logger(), "doc: %s\n",
                      doc.toJson().toStdString().c_str());

          QJsonObject jsonObj{doc.object()};
          QJsonArray bindingsArray =
              jsonObj["results"].toObject()["bindings"].toArray();

          std::vector<Object> vec{};
          for (auto &&obj : bindingsArray) {
            auto tmp{obj.toObject()};
            Object me{
                tmp["obj_id"].toObject()["value"].toString().toStdString(),
                tmp["class"].toObject()["value"].toString().toStdString(),
                std::stod(
                    tmp["x"].toObject()["value"].toString().toStdString()),
                std::stod(
                    tmp["y"].toObject()["value"].toString().toStdString()),
            };
            RCLCPP_INFO(get_logger(), "%s: %f, %f\n", me.uuid.c_str(), me.x,
                        me.y);
            if (me.klass == "human")
              vec.push_back(me);
          }
          RCLCPP_INFO(this->get_logger(), "Number of objects:%d", vec.size());

          generate_tst(std::move(vec));
        });
  };
};

int main(int argc, char **argv) {
  init(argc, argv);
  HumanDriver::SharedPtr node{std::make_shared<HumanDriver>()};
  spin(node);
  shutdown();
}
