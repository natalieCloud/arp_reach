cmake_minimum_required(VERSION 3.5)
project (arp_deployment_gp20)

# Set C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY model
    DESTINATION share/${PROJECT_NAME})

ament_package()
