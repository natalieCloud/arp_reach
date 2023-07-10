cmake_minimum_required(VERSION 3.5)
project(pcd_processing)

# Default to C++14
if (NOT CXX_CMAKE_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Set OS compiler standards
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arp_msgs REQUIRED)

add_executable(pcd_processor_client src/PoseArrayToPCDClient.cpp)
ament_target_dependencies(
    pcd_processeor_client
    rclcpp
    std_msgs
    geometry_msgs
    arp_msgs)

add_executable(pcd_processor_service src/PoseArrayToPCDService.cpp)
ament_target_dependencies(
    pcd_processeor_service
    rclcpp
    std_msgs
    geometry_msgs
    arp_msgs)

install(TARGETS
    pcd_processeor_client
    pcd_processeor_service
    DESTINATION lib/${PROJECT_NAME})

ament_package()
