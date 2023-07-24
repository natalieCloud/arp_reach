cmake_minimum_required(VERSION 3.5)
project (arp_deployment_gp20)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(DIRECTORY model
    DESTINATION share/${PROJECT_NAME})

ament_package()
