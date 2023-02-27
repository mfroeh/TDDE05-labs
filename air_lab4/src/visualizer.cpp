
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/detail/color_rgba__struct.hpp>
#include <vector>

#include <rclcpp/client.hpp>
#include <rclcpp/node.hpp>
#include <ros2_kdb_msgs/srv/detail/query_database__struct.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/detail/marker_array__struct.hpp>

#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

using namespace std::chrono_literals;
using namespace std::placeholders;
using namespace rclcpp;

using geometry_msgs::msg::Point;
using ros2_kdb_msgs::srv::QueryDatabase;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class Visualizer : public Node {
public:
  Visualizer() : Node{"Visualizer"} {
    publisher =
        create_publisher<MarkerArray>("/semantic_sensor_visualizer", 10);
    timer = create_wall_timer(500ms, std::bind(&Visualizer::query, this));
  }

private:
  Publisher<MarkerArray>::SharedPtr publisher;
  TimerBase::SharedPtr timer;
  Client<QueryDatabase>::SharedPtr client;

  std::string graphname{"semanticobject"};

  struct Object {
    std::string uuid;
    std::string klass;
    double x;
    double y;
  };

  void visualize(std::vector<Object> &&objects) {
    MarkerArray arr{};

    unsigned i{};
    for (auto &&obj : objects) {
      Marker m{};
      m.id = i++;
      m.header.frame_id = "odom";
      m.type = visualization_msgs::msg::Marker::CUBE;
      m.action = 0;
      m.scale.x = 0.5;
      m.scale.y = 0.5;
      m.scale.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.color.a = 1.0;

      Point p1{};
      p1.x = obj.x;
      p1.y = obj.y;
      m.points.push_back(p1);

      // Depending on class
      ColorRGBA color;
      color.r = 1.0;
      color.g = 0.5;
      color.b = 0.5;
      color.a = 1.0;
      m.colors.push_back(color);

      arr.markers.push_back(m);
    }

    publisher->publish(arr);
  }

  void query() {
    RCLCPP_INFO(this->get_logger(), "Starting query\n");
    auto request{
        std::make_shared<ros2_kdb_msgs::srv::QueryDatabase::Request>()};
    request->graphname = graphname;

    std::ostringstream os{};
    // os << "PREFIX gis: <http://www.ida.liu.se/~TDDE05/gis>" << std::endl
    //<< "PREFIX properties: <http://www.ida.liu.se/~TDDE05/properties>"
    //<< std::endl
    os << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class; ?x ?y }";
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

          auto objects{doc["results"]["bindings"].toArray()};
          std::vector<Object> vec{};
          for (auto &&obj : objects) {
            auto tmp{obj.toObject()};
            vec.push_back(Object{
                tmp["obj_id"].toObject()["value"].toString().toStdString(),
                tmp["class"].toObject()["value"].toString().toStdString(),
                tmp["x"].toObject()["value"].toDouble(),
                tmp["y"].toObject()["value"].toDouble(),
            });
          }
          RCLCPP_INFO(this->get_logger(), "Number of objects:%d", vec.size());

          visualize(std::move(vec));
        });
  };
};

int main(int argc, char **argv) {
  init(argc, argv);
  Visualizer::SharedPtr node{std::make_shared<Visualizer>()};
  spin(node);
  shutdown();
}
