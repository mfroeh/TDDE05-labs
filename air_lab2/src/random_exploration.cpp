#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class RandomExploration : public rclcpp::Node
{
  public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit RandomExploration()
      : Node("random_exploration")
    {
      this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
          this,
          "navigate_to_pose");

      this->timer_ = this->create_wall_timer(
          std::chrono::milliseconds(500),
          std::bind(&RandomExploration::send_goal, this));
    }

    void send_goal()
    {
      double x = 1.0;
      double y = 1.0;
      double angle = 1.0;

      auto goal_msg = NavigateToPose::Goal();
      goal_msg.pose.header.frame_id = "map";
      goal_msg.pose.pose.position.x = x;
      goal_msg.pose.pose.position.y = y;
      goal_msg.pose.pose.orientation.w = std::cos(angle/2);
      goal_msg.pose.pose.orientation.z = std::sin(angle/2);

      last_pos_ = goal_msg.pose.pose.position;

      using namespace std::placeholders;

      this->timer_->cancel();

      if (!this->client_ptr_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
        rclcpp::shutdown();
      }

      RCLCPP_INFO(this->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
      send_goal_options.goal_response_callback =
        std::bind(&RandomExploration::goal_response_callback, this, _1);
      send_goal_options.feedback_callback =
        std::bind(&RandomExploration::feedback_callback, this, _1, _2);
      send_goal_options.result_callback =
        std::bind(&RandomExploration::result_callback, this, _1);
      this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

  private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav2_msgs::msg::Point last_pos_;

    void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
    {
      auto goal_handle = future.get();
      if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
      }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr goal_handle,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      RCLCPP_INFO(this->get_logger(), "Got feedback");
      std::ostringstream oss{};
      oss << "Feedback " << feedback->current_pose.pose.position.x;
      auto lp{feedback->current_pose.pose.position};
      if (lp.x == last_pos_.x && lp.y == last_pos_.y && lp.z == last_pos_.z) {
        client_ptr_->async_cancel_goal(goal_handle);
        // set new goal
        /* send_goal(//something); */
      }
      last_pos_ = lp;

      if(feedback->navigation_time.sec>60)
      {
        client_ptr_->async_cancel_goal(goal_handle);
        RCLCPP_INFO(this->get_logger(), "Timeout");
        rclcpp::shutdown();
      }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result)
    {
      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(this->get_logger(), "Goal was succeeded");
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
};  // class RandomExploration

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RandomExploration>());
  rclcpp::shutdown();
  return 0;
}

