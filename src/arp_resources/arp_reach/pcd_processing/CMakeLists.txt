cmake_minimum_required(VERSION 3.5)
project(pcd_processing)

set(CASM_BOOST_NO_CXX11_SCOPED_ENUMS ON)

# Set OS compiler standards
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arp_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)

ament_python_install_package("pcd_processing")
ament_python_install_package("file_makers")


install(PROGRAMS
    nodes/PoseArrayToPCDClient.py
    nodes/PoseArrayToPCDService.py
DESTINATION lib/${PROJECT_NAME})

add_executable(pcd_service nodes/PoseArrayToPCDService.cpp
    file_makers_lib/src/PoseArrayToPCD.cpp
    file_makers_lib/src/YAMLFileWriter.cpp)
target_link_libraries(
    pcd_service
    Boost::system
    Boost::filesystem
    yaml-cpp
)
ament_target_dependencies(
    pcd_service
    rclcpp
    arp_msgs
    geometry_msgs
)

add_executable(pcd_client nodes/PoseArrayToPCDClient.cpp)
ament_target_dependencies(
    pcd_client
    rclcpp
    arp_msgs
    geometry_msgs
)

install(TARGETS
    pcd_service
    pcd_client
    DESTINATION lib/${PROJECT_NAME})

ament_package()