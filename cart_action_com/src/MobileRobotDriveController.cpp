#include <cnoid/SimpleController>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

#include "cart_action_interface/action/cart_action_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "action_tutorials_cpp/visibility_control.h"

class MobileRobotDriveController : public cnoid::SimpleController
{
public:
    using CartAction           = cart_action_interface::action::CartActionInterface;
    using GoalHandleCartAction = rclcpp_action::ServerGoalHandle<CartAction>;

    virtual bool configure(cnoid::SimpleControllerConfig* config) override;
    virtual bool initialize(cnoid::SimpleControllerIO* io) override;
    virtual bool control() override;

    MobileRobotDriveController()
    {
        node = rclcpp::Node::make_shared("mobile_robot_drive_controller");
    }

private:
    // for Choreonoid
    cnoid::Link* wheels[2];

    // for ROS2 Action communication
    rclcpp::Node::SharedPtr node;
    rclcpp_action::Server<CartAction>::SharedPtr action_server_;
    rclcpp::executors::StaticSingleThreadedExecutor::UniquePtr executor;

    geometry_msgs::msg::Twist command;
    std::shared_ptr<GoalHandleCartAction> goal_handle;

    ACTION_TUTORIALS_CPP_LOCAL
    void execute(const std::shared_ptr<GoalHandleCartAction> goal_handle)
    {
        RCLCPP_INFO(node->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(1);
        const auto goal = goal_handle->get_goal();
        auto feedback   = std::make_shared<CartAction::Feedback>();
        //auto & sequence = feedback->partial_sequence;
        auto & wheel_torque_vector = feedback->wheel_torque;
        auto result     = std::make_shared<CartAction::Result>();

        for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) 
        {
            // Confirm cancel request
            if (goal_handle->is_canceling()) 
            {
                //result->sequence = sequence;
                result->wheel_torque_vector = wheel_torque_vector;
                goal_handle->canceled(result);
                RCLCPP_INFO(node->get_logger(), "Goal canceled");
                return;
            }
            // Send Feedback to Action client
            /*
            sequence.push_back( wheels[0]->u() );
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(node->get_logger(), "Published wheel now data: %f", feedback->partial_sequence.back());
            */

            wheel_torque_vector.push_back( wheels[0]->u() );
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(node->get_logger(), "Published wheel now data: %f", feedback->wheel_torque.back());

            loop_rate.sleep();
        }

        // send result data
        if (rclcpp::ok()) 
        {
            // Stop cart
            command.linear.x  = 0.0;
            command.angular.z = 0.0;

            // Send Result to Action client
            //result->sequence = sequence;
            result->wheel_torque_vector = wheel_torque_vector;
            goal_handle->succeed(result);
            RCLCPP_INFO(node->get_logger(), "Goal succeeded");
        }
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(MobileRobotDriveController)

bool MobileRobotDriveController::configure(cnoid::SimpleControllerConfig* config)
{
    node = std::make_shared<rclcpp::Node>(config->controllerName());

    using namespace std::placeholders;

    // function that is executed when a "Goal" is received
    auto handle_goal = [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const CartAction::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(node->get_logger(), "Received goal request with order %d", goal->order);
        if (goal->order > 46)
        {
            return rclcpp_action::GoalResponse::REJECT;
        }

        command.linear.x = 0.5;
        command.angular.z = 1.0;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    // function that is executed when a "Cancel" is received
    auto handle_cancel = [this](const std::shared_ptr<GoalHandleCartAction> goal_handle)
    {
        RCLCPP_INFO(node->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };
    
    // function that is executed when a "Goal" is accepted
    auto handle_accepted = [this](const std::shared_ptr<GoalHandleCartAction> goal_handle)
    {
        auto execute_in_thread = [this, goal_handle]()
        {
            return this->execute(goal_handle);
        };
        std::thread{execute_in_thread}.detach();
    };
    
    this->action_server_ = rclcpp_action::create_server<CartAction>(
        node            , // node pointer
        "CartAction"    , // name of action
        handle_goal     , // function that is executed when a "Goal" is received
        handle_cancel   , // function that is executed when a "Cancel" is received
        handle_accepted   // function that is executed when a "Goal" is accepted
        );

    executor = std::make_unique<rclcpp::executors::StaticSingleThreadedExecutor>();
    executor->add_node(node);

    return true;
}

bool MobileRobotDriveController::initialize(cnoid::SimpleControllerIO* io)
{
    auto body = io->body();
    wheels[0] = body->joint("LeftWheel");
    wheels[1] = body->joint("RightWheel");

    for(int i=0; i < 2; ++i)
    {
        auto wheel = wheels[i];
        wheel->setActuationMode(JointTorque);
        io->enableInput(wheel, JointVelocity);
        io->enableOutput(wheel, JointTorque);
    }
    return true;
}

bool MobileRobotDriveController::control()
{
    executor->spin_some();

    constexpr double wheelRadius = 0.076;
    constexpr double halfAxleWidth = 0.145;
    constexpr double kd = 0.5;

    double dq_x   = command.linear.x / wheelRadius;
    double dq_yaw = command.angular.z * halfAxleWidth / wheelRadius;

    double dq_target[2];
    dq_target[0] = dq_x - dq_yaw;
    dq_target[1] = dq_x + dq_yaw;
    
    for(int i=0; i < 2; ++i)
    {
        auto wheel = wheels[i];
        wheel->u() = kd * (dq_target[i] - wheel->dq());
    }

    return true;
}

