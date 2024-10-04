cmake_minimum_required(VERSION 3.5)
project(stanley_carla)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(carla_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)  # 추가된 부분

add_executable(stanley_carla src/stanley.cpp)
ament_target_dependencies(stanley_carla rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs visualization_msgs)  # 수정된 부분

add_executable(control_node src/control.cpp)
ament_target_dependencies(control_node rclcpp geometry_msgs carla_msgs)

install(TARGETS
  stanley_carla
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Add testing targets here
endif()

ament_package()