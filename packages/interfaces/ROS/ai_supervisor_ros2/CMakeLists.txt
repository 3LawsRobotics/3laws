cmake_minimum_required(VERSION 3.15)
project(ai_supervisor_ros2)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(lll_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(AISupervisor 0.3.0 REQUIRED)

# ---------------------------------------------------------------------------------------
# COMPONENTS
# ---------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_components SHARED
  src/kernel_generator_node.cpp
  src/input_filter_node.cpp
  src/kernel_generator_node_unicycle.cpp
)
target_include_directories(${PROJECT_NAME}_components
PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}_components
PUBLIC
  rclcpp::rclcpp
  3laws::AISupervisor-shared
  lll_msgs::lll_msgs__rosidl_typesupport_cpp
PRIVATE
  Boost::headers
  Eigen3::Eigen
)

target_compile_options(${PROJECT_NAME}_components PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# ---------------------------------------------------------------------------------------
# EXECUTABLE
# ---------------------------------------------------------------------------------------

add_executable(kernel_generator_unicycle
  src/kernel_generator_unicycle.cpp
)
target_link_libraries(kernel_generator_unicycle
  ${PROJECT_NAME}_components
)
target_compile_options(kernel_generator_unicycle PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

add_executable(input_filter
  src/input_filter.cpp
)
target_link_libraries(input_filter
  ${PROJECT_NAME}_components
)
target_compile_options(input_filter PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# ---------------------------------------------------------------------------------------
# INSTALLATION
# ---------------------------------------------------------------------------------------

install(
  TARGETS               ${PROJECT_NAME}_components kernel_generator_unicycle input_filter
  EXPORT                export_${PROJECT_NAME}
  ARCHIVE DESTINATION   lib
  LIBRARY DESTINATION   lib
  RUNTIME DESTINATION   lib/${PROJECT_NAME}
)

install(
  DIRECTORY     include/
  DESTINATION   include/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ---------------------------------------------------------------------------------------
# TESTING
# ---------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/../../.clang-format")

  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_clang_format
    ament_cmake_cpplint
    ament_cmake_cppcheck
  )
  ament_lint_auto_find_test_dependencies()

  file(GLOB_RECURSE GOOD_INCLUDE "include/*")
  file(GLOB_RECURSE GOOD_SRC "src/*")
  list(FILTER GOOD_SRC EXCLUDE REGEX "${CMAKE_CURRENT_SOURCE_DIR}/src/external/.*")
  list(APPEND GOOD_SRC ${GOOD_INCLUDE})

  ament_clang_format(${GOOD_SRC})
  ament_cpplint(${GOOD_SRC} FILTERS "-build/header_guard")
  ament_cppcheck(${GOOD_SRC})
endif()

# ---------------------------------------------------------------------------------------
# CMAKE CONFIGURATION
# ---------------------------------------------------------------------------------------

ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp lll_msgs Boost Eigen3 AISupervisor)
ament_package()
