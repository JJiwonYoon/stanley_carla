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
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)  # 추가된 부분

# include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${carla_msgs_INCLUDE_DIRS}
  ${visualization_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}  # 추가된 부분
)

# add executables
add_executable(stanley_carla src/stanley.cpp)
ament_target_dependencies(stanley_carla rclcpp sensor_msgs nav_msgs std_msgs geometry_msgs visualization_msgs)

add_executable(control_node src/control.cpp)
ament_target_dependencies(control_node rclcpp geometry_msgs carla_msgs)

# install targets
install(TARGETS
  stanley_carla
  control_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # Add testing targets here
endif()

ament_package()