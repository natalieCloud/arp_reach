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

# Set C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

set(CASM_BOOST_NO_CXX11_SCOPED_ENUMS ON)

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
find_package(Boost COMPONENTS system filesystem REQUIRED)

# External Packages
find_package(arp_deployment_gp20 REQUIRED)
find_package(yaml-cpp REQUIRED)

# Reach dependent packages
set(REACH_DEPS
    tf2_eigen
    interactive_markers
    moveit_core
    moveit_msgs
    moveit_ros_planning_interface
    sensor_msgs
    visualization_msgs
    std_msgs
    arp_msgs
    boost_plugin_loader)

foreach(dep ${REACH_DEPS})
    find_package(${dep} REQUIRED)
endforeach()

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

add_executable(reach_service nodes/reachStudyService.cpp)
target_link_libraries(reach_service PUBLIC reach::reach yaml-cpp Boost::system Boost::filesystem)
ament_target_dependencies(reach_service PUBLIC ${REACH_DEPS} rclcpp reach_ros reach)

add_executable(reach_client nodes/reachClient.cpp)
target_link_libraries(
    reach_client
    Boost::system
    Boost::filesystem
)
ament_target_dependencies(reach_client rclcpp arp_msgs reach_ros reach)

install(TARGETS
    reach_service
    reach_client
    DESTINATION lib/${PROJECT_NAME})


ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
    scripts/reach_client_test.py
    DESTINATION lib/${PROJECT_NAME})
    
ament_package()