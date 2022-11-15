cmake_minimum_required(VERSION 3.15)
project(lll_msgs)

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # messages
  msg/Float64VectorStamped.msg
  msg/InputConstraints.msg
  msg/InputFilteringResult.msg
  msg/RegulationData.msg

  # services
  srv/InputFiltering.srv

DEPENDENCIES
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
  std_msgs
  std_srvs
  geometry_msgs
  nav_msgs
)
ament_package()

