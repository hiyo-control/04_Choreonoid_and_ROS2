#include <memory>
#include <string>
#include <sstream>

#include "cart_action_interface/action/cart_action_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

namespace action_tutorials_cpp
{
class CartActionClient : public rclcpp::Node
{
public:
  using CartAction           = cart_action_interface::action::CartActionInterface;
  using GoalHandleCartAction = rclcpp_action::ClientGoalHandle<CartAction>; 

  ACTION_TUTORIALS_CPP_PUBLIC
  explicit CartActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()): Node("cart_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<CartAction>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "CartAction");

    this->timer_ = this->create_wall_timer(std::chrono::milliseconds(500), [this]() 
    {
      return this->send_goal();
    });
  }

  ACTION_TUTORIALS_CPP_PUBLIC
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) 
    {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }
    auto goal_msg = CartAction::Goal();
    goal_msg.order = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<CartAction>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](const GoalHandleCartAction::SharedPtr & goal_handle)
      {
        if (!goal_handle) 
        {
          RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } 
        else 
        {
          RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
      };

    send_goal_options.feedback_callback = [this](GoalHandleCartAction::SharedPtr,const std::shared_ptr<const CartAction::Feedback> feedback)
      {
        RCLCPP_INFO(this->get_logger(), "Subscribed wheel now data : %f", feedback->partial_sequence.back());
      };

    send_goal_options.result_callback = [this](const GoalHandleCartAction::WrappedResult & result)
      {
        switch (result.code) 
        {
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
        std::stringstream ss;
        ss << "Result received: ";
        for (auto number : result.result->sequence)
        {
          ss << number << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
        rclcpp::shutdown();
      };

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<CartAction>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
};  // class CarttActionClient

}  // namespace action_tutorials_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::CartActionClient)
