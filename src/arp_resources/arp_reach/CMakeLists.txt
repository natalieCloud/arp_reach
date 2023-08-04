cmake_minimum_required(VERSION 3.5)
project(arp_reach_launch)

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

# Find build type
find_package(ament_cmake REQUIRED)

find_package(arp_msgs REQUIRED)

# Find Submodules
add_subdirectory (pcd_processing)
add_subdirectory (xml_processing)
add_subdirectory (arp_reach)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()