#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::placeholders;

namespace action_tutorials_cpp
{
  class FibonacciActionClient : public rclcpp::Node
  {
    public:
      using Fibonacci = nav2_msgs::action::NavigateToPose;
      using GoalHandleFibonacci = rclcpp_action::ClientGoalHandle<Fibonacci>;

      explicit FibonacciActionClient()
        : Node("fibonacci_action_client")
      {
        this->client_ptr_ = rclcpp_action::create_client<Fibonacci>(
            this,
            "fibonacci"
            );
      }

      void send_goal(double x, double y, double angle)
      {
        if (!this->client_ptr_->wait_for_action_server()) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          rclcpp::shutdown();

        }

        auto goal_msg = Fibonacci::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.pose.position.x = x;
        goal_msg.pose.pose.position.y = y;
        goal_msg.pose.pose.orientation.w = std::cos(angle/2);
        goal_msg.pose.pose.orientation.z = std::sin(angle/2);

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<Fibonacci>::SendGoalOptions();
        send_goal_options.goal_response_callback =
          std::bind(&FibonacciActionClient::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
          std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
          std::bind(&FibonacciActionClient::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

      }

    private:
      rclcpp_action::Client<Fibonacci>::SharedPtr client_ptr_;

      void goal_response_callback(std::shared_future<GoalHandleFibonacci::SharedPtr> future)
      {
        auto goal_handle = future.get();
        if (!goal_handle) {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");

        } else {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");

        }
        //TODO copy the thing and then maybe we don't need it
        //        self._get_result_future = goal_handle.get_result_async()
        //                self._get_result_future.add_done_callback(self.get_result_callback)
      }

      void feedback_callback(
          GoalHandleFibonacci::SharedPtr handle,
          const std::shared_ptr<const Fibonacci::Feedback> feedback
          )
      {
        std::stringstream ss;
        ss << "Received feedback: " << feedback->navigation_time.sec << std::endl;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
        if(feedback->navigation_time.sec > 60) {
          client_ptr_->async_cancel_goal(handle, [this](auto) {
            RCLCPP_INFO(this->get_logger(), "Testor");
          });
          rclcpp::shutdown();
        }
      }

      void result_callback(const GoalHandleFibonacci::WrappedResult & result)
      {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
          default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;

        }
        rclcpp::shutdown();
      }

  };  // class FibonacciActionClient


}  // namespace action_tutorials_cpp
using action_tutorials_cpp::FibonacciActionClient;

int main(int argc, char** argv) {

  rclcpp::init(argc, argv);
  std::shared_ptr<FibonacciActionClient> fibo{};
  rclcpp::spin(fibo);
  fibo->send_goal(10, 5, 1);

  rclcpp::shutdown();
}
