cmake_minimum_required(VERSION 3.5)
project(pcd_processing)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arp_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

ament_python_install_package("pcd_processing")
ament_python_install_package("file_makers")

install(PROGRAMS
    nodes/PoseArrayToPCDClient.py
    nodes/PoseArrayToPCDService.py
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()