cmake_minimum_required(VERSION 3.5)
project(arp_reach)

# Set OS Compiler Standards
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++14
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()
# Set C++ flags
set(CMAKE_CXX_FLAGS -rdynamic)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

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
    rclcpp
    sensor_msgs
    visualization_msgs
    std_msgs
    arp_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

foreach(dep ${ROS2_DEPS})
    find_package(${dep} REQUIRED)
endforeach()

add_executable(reach_service nodes/reach_study_node.cpp)
target_link_libraries(reach_service PUBLIC reach::reach yaml-cpp)
ament_target_dependencies (reach_service PUBLIC ${ROS2_DEPS})

ament_python_install_package("arp_reach")

install(TARGETS
    reach_service
    DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
    nodes/reach_client_test.py
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})
    
ament_package()