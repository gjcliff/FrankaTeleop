cmake_minimum_required(VERSION 3.8)
project(franka_teleop)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(rosidl_default_generators REQUIRED)

add_executable(franka_teleop_node src/franka_teleop.cpp)

rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/PlanPath.srv"
)
ament_export_dependencies(rosidl_default_runtime)

ament_target_dependencies(
  franka_teleop_node
  rclcpp
  geometry_msgs
  std_srvs
  moveit
  moveit_visual_tools
)


install(TARGETS
    franka_teleop_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY
    launch/
    DESTINATION share/${PROJECT_NAME}/launch
)


rosidl_get_typesupport_target(cpp_typesupport_target
    ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(franka_teleop_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights

  # comment the line when a copyright and license is added to all source 
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
