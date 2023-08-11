cmake_minimum_required(VERSION 3.5)

project("xml_processing")

#Set to C99 Standard
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Set C++17 Standard
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(arp_msgs REQUIRED)
find_package(Boost COMPONENTS system filesystem REQUIRED)

# Add Executables
add_executable(xml_parse_service nodes/XMLToPoseArrayService.cpp 
    parsing_lib/src/xml_parser.cpp 
    parsing_lib/src/result_parsing.cpp
    parsing_lib/src/array_transform.cpp
    parsing_lib/src/float_standard.cpp)
ament_target_dependencies (
    xml_parse_service
    rclcpp
    geometry_msgs
    arp_msgs
    std_msgs
)

add_executable(xml_parse_client nodes/XMLToPoseArrayClient.cpp)
target_link_libraries(
    xml_parse_client
    Boost::system
    Boost::filesystem
)
ament_target_dependencies (
    xml_parse_client
    rclcpp
    geometry_msgs
    arp_msgs
    std_msgs
)

# Install
install(TARGETS
    xml_parse_service
    xml_parse_client
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
