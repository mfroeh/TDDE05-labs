#include "air_lab_interfaces/srv/execute_tst.hpp"

#include <cmath>
#include <functional>
#include <memory>
#include <sstream>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <string>
#include <vector>

#include "air_lab_interfaces/msg/goal.hpp"
#include "air_lab_interfaces/msg/goals.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <ros2_kdb_msgs/srv/detail/query_database__struct.hpp>

#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

using namespace std::chrono_literals;

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

using air_lab_interfaces::msg::Goal;
using air_lab_interfaces::msg::Goals;

using namespace rclcpp;

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node{"decision"} {
    subscription_ =
        create_subscription<Goals>("goals_requests", 10,
                                   std::bind(&DecisionNode::decision_callback,
                                             this, std::placeholders::_1));
  }

  struct Object {
    std::string uuid;
    std::string klass;
    std::string tag;
    double x;
    double y;
  };

private:
  rclcpp::Subscription<Goals>::SharedPtr subscription_;

  void decision_callback(const Goals::SharedPtr msg) {
    RCLCPP_INFO(get_logger(), "Got goal request for destination: %s", msg->goals[0].destination.c_str());

    std::vector<Goal> goals{msg->goals};

    RCLCPP_INFO(this->get_logger(), "Starting query\n");
    auto request{
        std::make_shared<ros2_kdb_msgs::srv::QueryDatabase::Request>()};
    request->graphname = "semanticobject";
    request->format = "json";

    std::ostringstream os{};
    os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>\n"
        << "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>\n"
        << "SELECT ?obj_id ?class ?tags ?x ?y WHERE { ?obj_id a ?class ; "
        << "properties:location [ gis:x ?x; gis:y ?y ] ; properties:tags "
        << "?tags . }\n";

    request->query = os.str();
    auto client = create_client<ros2_kdb_msgs::srv::QueryDatabase>("/kdb_server/sparql_query");
    client->wait_for_service();
    RCLCPP_INFO(get_logger(), "Service online!\n");
    auto response = client->async_send_request(
            request, [this, goals](Client<ros2_kdb_msgs::srv::QueryDatabase>::SharedFuture future) {
            QJsonDocument document{QJsonDocument::fromJson(
                    QByteArray::fromStdString(future.get()->result))};

            RCLCPP_INFO(this->get_logger(), "Received result\n");

            RCLCPP_INFO(this->get_logger(), "Object: %s\n",
                    future.get()->result.c_str());
            if (!future.get()->success) {
            RCLCPP_ERROR(get_logger(),
                    "Query: /kdb_server/sparql_query wasn't successful!");
            return;
            }

            RCLCPP_INFO(this->get_logger(), "document: %s\n",
                    document.toJson().toStdString().c_str());

            QJsonObject jsonObj{document.object()};
            QJsonArray bindingsArray =
            jsonObj["results"].toObject()["bindings"].toArray();

            std::vector<Object> vec{};
            for (auto &&obj : bindingsArray) {
                auto tmp{obj.toObject()};
                Object me{
                    tmp["obj_id"].toObject()["value"].toString().toStdString(),
                        tmp["class"].toObject()["value"].toString().toStdString(),
                        tmp["tags"].toObject()["value"].toString().toStdString(),
                        std::stod(
                                tmp["x"].toObject()["value"].toString().toStdString()),
                        std::stod(
                                tmp["y"].toObject()["value"].toString().toStdString()),
                };
                RCLCPP_INFO(get_logger(), "%s: %f, %f\n", me.uuid.c_str(), me.x,
                        me.y);
                vec.push_back(me);
            }
            RCLCPP_INFO(this->get_logger(), "Number of objects:%d", vec.size());

            QJsonObject seq_node;
            seq_node.insert("name", "seq");
            seq_node.insert("common_params", QJsonObject());
            seq_node.insert("params", QJsonObject());

            QJsonArray children;

            // Handle goals
            auto goal{goals[0]};
            if (goal.type == "goto") {
                auto found = std::find_if(
                        std::begin(vec), std::end(vec), [&goal](Object obj) {
                        return (obj.klass == "office" && obj.tag == goal.destination);
                        });
                if (found != std::end(vec)) {
                    // Add the drive_to node to the children array
                    children.append(make_drive_to_node(*found));
                }
            } else if (goal.type == "bring") {
                auto vending = std::find_if(
                        std::begin(vec), std::end(vec),
                        [&goal](Object obj) { return (obj.tag == goal.object); });
                auto guy = std::find_if(
                        std::begin(vec), std::end(vec), [&goal](Object obj) {
                        return (obj.klass == "office" && obj.tag == goal.destination);
                        });
                if (vending != vec.end() && guy != vec.end()) {
                    // Add the drive_to node to the children array
                    children.append(make_drive_to_node(*vending));
                    children.append(make_drive_to_node(*guy));
                }
            } else {
                QJsonObject jsonObj{
                    {"children", QJsonArray()},
                        {"common_params", QJsonObject()},
                        {"name", "explore"},
                        {"params", QJsonObject{{"radius", 5}, {"a", 0}, {"b", 1}}}};
                children.append(jsonObj);
            }

            seq_node.insert("children", children);
            // Create the JSON document
            QJsonDocument doc{seq_node};

            // Write the document to a file named 'tst.json'
            QFile file("goal_tst.json");
            if (file.open(QIODevice::WriteOnly)) {
                file.write(doc.toJson());
                file.close();
            }
            auto client =
                create_client<air_lab_interfaces::srv::ExecuteTst>("execute_tst");
            RCLCPP_INFO(this->get_logger(), "Waiting on service");
            client->wait_for_service();
            RCLCPP_INFO(this->get_logger(), "Service online\n");

            auto request{
                std::make_shared<air_lab_interfaces::srv::ExecuteTst::Request>()};
            request->tst_file = "goal_tst.json";
            auto response = client->async_send_request(request);
            });
  }

  QJsonObject make_drive_to_node(Object guy) {
      QJsonArray drive_to_children;
      QJsonObject params;

      // Create the Point object for the 'p' parameter
      QJsonObject p;
      p.insert("rostype", "Point");
      p.insert("x", guy.x);
      p.insert("y", guy.y);
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

      return drive_to_node;
  }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DecisionNode>());
    rclcpp::shutdown();
    return 0;
}
