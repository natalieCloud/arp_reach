# ARP_Reach README

<a href="https://docs.ros.org/en/humble/index.html"><img src="https://img.shields.io/badge/ROS 2-Humble-blue"/></a>
[![Github Issues](htpps://img.shields.io/github/issues/natalieCloud/arp_reach.svg)](https://github.com/natalieCloud/arp_reach/issues)

<!--ARP_Reach (*augmented_reality_painting_reach_integration*) is a package that enables the integration of [Ros-Industrial-Reach](https://github.com/ros-industrial/reach) as well as [Ros_Industrial-Reach_Ros2](https://github.com/ros-industrial/reach_ros2) with the OSU-AIMS [augmented_reality_painting](https://github.com/OSU-AIMS/augmented-reality-painting) project! -->

This package is orginized around creating and supprting the integration of `reach_ros` through three main client service pairs and their support functions. The services are found as [PCD_Processing](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing) , [ARP_Reach](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/arp_reach), and [XML_Processing](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing) which process a data type of `PoseArray` to a pcd file, run that pcd file through `Reach` to get the results database, and process that database back into a list full of the `PoseArray`'s scores, respectively. More information can be found in each of their respective README's for information on implementation

## Installation and Build 

To sucessfully build this project one must have minimum [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed, as well as [python3](https://www.python.org/downloads/) configs set up!

### Installation

First, clone the repository:

```
cd ~/path/to/workspace/src
git clone https://github.com/natalieCloud/arp_reach.git
cd ..
```
Ensure submodules are pulled down and updated! (Check dependencies.repos for most recent working commit of each submodule!)

Install those dependencies! (Reach, Reach_Ros, Boost, etc)

```
vcs import src < src/arp_reach/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
```

### Build

To build this package first navigate to the base directory and call:

```
    source /opt/ros/humble/setup.bash
```

Then navigate towards the root of the workspace and build the project:

```
    cd /path/to/workspace/src
    colcon build --symlink-install
```

Once the package has finished building make sure to source the install

```
    source ./install/setup.bash 
```

### Executables

[**ARP_Reach_Launch**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/launch)
- reach_launch.xml: Launches all of the service nodes from `PCD_Processing`, `XML_Processing`, and `ARP_Reach`!

[**arp_msgs**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_msgs) 
- None Y(^u^)Y

[**arp_deployment_gp20**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_deployment_gp20)
- None Y(^w^)Y

[**arp_reach**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/arp_reach)
- reachStudyService
- reachClient
  
[**pcd_processing**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing) 
- PoseArrayToPCDClient
- PoseArrayToPCDService

[**xml_processing**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing)
- xml_parse_service
- xml_parse_client

### Structure

In the main launch file, the main nodes that will be called will be laid out as follows:

![image](https://github.com/natalieCloud/arp_reach/assets/123828141/06f67bf9-33dd-4679-8c94-e859169b7091)

Important to note: Each box with sidebars represents a node or launch file that will be called from inside the regular main "Node Hub Launch" file! The dashed lines represent helper functions and the single line arrow inputs, hollow arrows output! 
