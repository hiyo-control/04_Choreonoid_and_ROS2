cmake_minimum_required(VERSION 3.8)
project(cart_action_com)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cart_action_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(choreonoid REQUIRED)

include_directories(include)
include_directories(${CHOREONOID_INCLUDE_DIRS})

link_directories(${CHOREONOID_LIBRARY_DIRS})

add_subdirectory(src)

### Action server ###
add_library(server_node SHARED
  src/MobileRobotDriveController.cpp
)
target_compile_definitions(server_node
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL"
)
ament_target_dependencies(server_node
  "cart_action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
)

### Action client ###
add_library(client_node SHARED
  src/cart_action_client.cpp
)
rclcpp_components_register_node(client_node PLUGIN "action_tutorials_cpp::CartActionClient" EXECUTABLE cart_action_client)
target_compile_definitions(client_node
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL"
)
ament_target_dependencies(client_node
  "cart_action_interface"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "std_msgs"
)

install(TARGETS
  server_node
  client_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
