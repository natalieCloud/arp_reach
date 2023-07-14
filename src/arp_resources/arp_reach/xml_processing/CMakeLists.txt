cmake_minimum_required(VERSION 3.5)

project("xml_processing")

#Set to C99 Standard
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Set C++14 Standard
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arp_msgs REQUIRED)

# Add Executables
add_executable(testing nodes/XMLToPoseArrayService.cpp)
ament_target_dependencies (
    testing
    rclcpp
    geometry_msgs
    arp_msgs
    std_msgs
)

# Install
install(TARGETS
    testing
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
