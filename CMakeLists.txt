cmake_minimum_required(VERSION 3.5)
project(move_base2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# add_compile_definitions(-Wunused-parameter)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_controller REQUIRED)
find_package(std_srvs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)
find_package(nav_2d_utils REQUIRED)
find_package(angles REQUIRED)
find_package(map_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rcl_interfaces REQUIRED)
find_package(athena_interfaces REQUIRED)

## dependencies
set(dependencies
  rclcpp
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav2_common
  nav2_costmap_2d
  nav2_controller
  nav_msgs
  nav_2d_msgs
  nav_2d_utils
  geometry_msgs
  sensor_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
  nav2_core
  angles
  map_msgs
  rcl_interfaces
  athena_interfaces
)

## include directory
include_directories(include)

## srv
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/NavigateToPose.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

## node
add_executable(move_base_node src/main.cpp  src/MoveBase.cpp src/PointCost.cpp)
target_include_directories(move_base_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(move_base_node ${dependencies})
target_compile_definitions(move_base_node PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS") # prevent pluginlib from using boost
rosidl_target_interfaces(move_base_node ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序


#lifecycle_client
add_executable(lifecycle_client src/lifecycle_client.cpp)
target_include_directories(lifecycle_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(lifecycle_client ${dependencies})
target_compile_definitions(lifecycle_client PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS") # prevent pluginlib from using boost


## srv client
add_executable(navi_to_client src/navi_to_client.cpp)
target_include_directories(navi_to_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(navi_to_client ${dependencies})
target_compile_definitions(navi_to_client PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS") # prevent pluginlib from using boost
rosidl_target_interfaces(navi_to_client ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序


## object tracking msg transfer
add_executable(receive_goal_to_mb src/main_to_mb.cpp src/receive_goal_to_mb.cpp)
ament_target_dependencies( receive_goal_to_mb  ${dependencies})
rosidl_target_interfaces(receive_goal_to_mb ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序

## costmap pub cycle test
add_executable(test1 test/test.cpp)
ament_target_dependencies(test1  ${dependencies})
rosidl_target_interfaces(test1 ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序

## costmap pub cycle test
add_executable(portal test/namespace_portal.cpp)
ament_target_dependencies(portal  ${dependencies})
rosidl_target_interfaces(portal ${PROJECT_NAME} "rosidl_typesupport_cpp")  # 支持action定义的依赖性顺序

## install


install(TARGETS move_base_node lifecycle_client navi_to_client receive_goal_to_mb portal test1
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch maps params
  DESTINATION share/${PROJECT_NAME}
)

## declare
ament_export_dependencies(rosidl_default_runtime)
ament_package()
