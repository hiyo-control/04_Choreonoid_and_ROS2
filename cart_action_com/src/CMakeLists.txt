choreonoid_add_simple_controller(
    MobileRobotDriveController
    MobileRobotDriveController.cpp
    )

ament_target_dependencies(
    MobileRobotDriveController
    geometry_msgs
    rclcpp
    rclcpp_components
    rclcpp_action
    cart_action_interface
    )


