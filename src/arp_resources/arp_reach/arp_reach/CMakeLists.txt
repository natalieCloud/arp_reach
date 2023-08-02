cmake_minimum_required(VERSION 3.5)
project(arp_reach)

# Set OS Compiler Standards
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Set C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Set C++ flags
set(CMAKE_CXX_FLAGS -rdynamic)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

# Find Submodules
find_package(reach REQUIRED)
find_package(reach_ros REQUIRED)
find_package(boost_plugin_loader REQUIRED)

# External Packages
find_package(arp_deployment_gp20 REQUIRED)
find_package(yaml-cpp REQUIRED)

# ROS 2 dependent packages
set(ROS2_DEPS
    tf2_eigen
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    sensor_msgs
    visualization_msgs
    std_msgs
    arp_msgs)

foreach(dep ${ROS2_DEPS})
    find_package(${dep} REQUIRED)
endforeach()

install(DIRECTORY launch config 
    DESTINATION share/${PROJECT_NAME})

add_executable(reach_service nodes/reach_study_node.cpp)
target_link_libraries(reach_service PUBLIC reach::reach yaml-cpp)
ament_target_dependencies(reach_service PUBLIC ${ROS2_DEPS} rclcpp reach_ros reach boost_plugin_loader)

add_executable(reach_client nodes/reach_client.cpp)
ament_target_dependencies(reach_client rclcpp arp_msgs reach_ros reach boost_plugin_loader)

install(TARGETS
    reach_service
    reach_client
    DESTINATION lib/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
    scripts/reach_client_test.py
    DESTINATION lib/${PROJECT_NAME})
    
ament_package()