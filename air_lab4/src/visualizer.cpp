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

class Visualizer : public Node {
public:
  Visualizer() : Node{"Visualizer"} {
    publisher =
        create_publisher<MarkerArray>("/semantic_sensor_visualizer", 10);
    timer = create_wall_timer(500ms, std::bind(&Visualizer::query, this));

    // The tf_buffer and tf_listener needs to be kept alive and should be
    // created in a constructor
    tf_buffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  Publisher<MarkerArray>::SharedPtr publisher;
  TimerBase::SharedPtr timer;
  Client<QueryDatabase>::SharedPtr client;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;

  std::string graphname{"semanticobject"};

  struct Object {
    std::string uuid;
    std::string klass;
    double x;
    double y;
  };

  void visualize(std::vector<Object> &&objects) {
    MarkerArray arr{};

    Marker marker{};
    marker.id = 1242;
    marker.header.frame_id = "odom";
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = 0;
    marker.scale.x = 5.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.orientation.w = 1.0;
    marker.color.a = 1.0;

    for (auto &&obj : objects) {
      RCLCPP_INFO(get_logger(), "%s: %f, %f\n", obj.uuid.c_str(), obj.x, obj.y);
      PointStamped stamped{};
      stamped.header.frame_id = "turtlebot0/semantic_sensor";
      stamped.point.x = obj.x;
      stamped.point.y = obj.y;

      PointStamped p_after{};
      try {
        p_after = tf_buffer->transform(stamped, "map");
      } catch (const tf2::TransformException &ex) {
          RCLCPP_INFO(get_logger(), "Could not transform %s to %s: %s",
            stamped.header.frame_id.c_str(), "map", ex.what());
        continue;
      }

      marker.points.push_back(p_after.point);

      // Depending on class
      ColorRGBA color;
      color.a = 1.0;
      if (obj.klass == "table") {
        color.r = 1.0;
        color.g = 0.5;
        color.b = 0.5;
      } else {
        color.r = 0.5;
        color.g = 1.0;
        color.b = 0.5;
      }
      marker.colors.push_back(color);
    }
    arr.markers.push_back(marker);

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
    // os << "SELECT ?obj_id ?class ?x ?y WHERE { ?obj_id a ?class; ?x ?y }";

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
            vec.push_back(me);
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
