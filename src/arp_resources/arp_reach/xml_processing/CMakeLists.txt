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

#Add Executables
add_executable(testing src/XMLToPoseArrayService.cpp)
ament_target_dependencies (
    testing
    rclcpp
)

# Install
install(TARGETS
    testing
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
