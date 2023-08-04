#!/bin/bash
set -e

# Color Color Color
RED="\e[31m"
GREEN="\e[32m"
ENDCOLOR="\[0m" 

# Paths
ROS="/opt/ros/humble/setup.bash"
OVERLAY_ARP="../../install/setup.bash"

# Source
echo -e "${GREEN} > Sourcing ROS:${ENDCOLOR} ${ROS}"
source ${ROS}
echo -e "${GREEN} > Sourcing worskpace:${ENDCOLOR} ${OVERLAY_ARP}"
source ${OVERLAY_ARP}

BUILD_TYPE=RelWithDebInfo

echo -e "${GREEN} > Starting build with colcon.${ENDCOLOR}"
colcon build \
    --symlink-install \
    --cmake-args "DCMAKE_BUILD_TYPE=$BUILD_TYPE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=On" \
    # -Wall Wextra Wpedantic

echo -e "${GREEN} > Build Complete.${ENDCOLOR}"
#EOF