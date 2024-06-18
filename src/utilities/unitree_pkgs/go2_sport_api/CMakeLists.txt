cmake_minimum_required(VERSION 3.5)
project(go2_sport_api)

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


include_directories(include include/common include/nlohmann)
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_api
  rclcpp
  std_msgs
  # rosbag2_cpp
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rosbag2_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vel_ctrl src/vel_ctrl_repub.cpp src/common/ros2_sport_client.cpp)
add_library(go2_sport_api SHARED src/common/ros2_sport_client.cpp)

ament_target_dependencies(vel_ctrl ${DEPENDENCY_LIST})
ament_target_dependencies(go2_sport_api ${DEPENDENCY_LIST})

install(
        DIRECTORY include
        DESTINATION include
)

install(TARGETS vel_ctrl
        DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS go2_sport_api EXPORT export_go2_sport_api
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_dependencies(unitree_go unitree_api sensor_msgs)
ament_export_libraries(go2_sport_api)
ament_export_include_directories(include)
ament_export_targets(export_go2_sport_api)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
