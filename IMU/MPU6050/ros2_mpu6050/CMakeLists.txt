cmake_minimum_required(VERSION 3.5)
project(mpu6050driver)

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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(mpu6050driver src/mpu6050driver.cpp src/mpu6050sensor.cpp)
ament_target_dependencies(mpu6050driver rclcpp std_msgs sensor_msgs tf2 tf2_ros tf2_geometry_msgs Eigen3)
target_link_libraries(mpu6050driver i2c)
target_include_directories(
  mpu6050driver PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

install(TARGETS mpu6050driver DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)
ament_package()
