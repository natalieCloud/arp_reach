cmake_minimum_required(VERSION 3.5)
project(reach_config)

# Set OS Compiler Standards
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(arp_msgs)

find_package(arp_deployment_gp20 REQUIRED)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()