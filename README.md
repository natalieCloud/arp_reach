# ARP_Reach README

<a href="https://docs.ros.org/en/humble/index.html"><img src="https://img.shields.io/badge/ROS 2-Humble-blue"/></a>
[![Github Issues](htpps://img.shields.io/github/issues/natalieCloud/arp_reach.svg)](https://github.com/natalieCloud/arp_reach/issues)

ARP_Reach (*augmented_reality_painting_reach_integration*) is a package that enables the integration of [Ros-Industrial-Reach](https://github.com/ros-industrial/reach) as well as [Ros_Industrial-Reach_Ros2](https://github.com/ros-industrial/reach_ros2) with the OSU-AIMS [augmented_reality_painting](https://github.com/OSU-AIMS/augmented-reality-painting) project! 

This package is orginized around 3 main ROS services and their support files, the services are found as [PCD_PROCESSING](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing) , [Reach_Config](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/reach_config), and [XML_PROCESSING](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing) which process a data type of posearray to a pcd file, run that pcd file through reach to get the results database, and process that database back into a list full of the poseArray's scores, respectively. More information can be found in each of their respective README's for information on implementation

## Installation and Build 

To sucessfully build this project one must have minimum [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed, as well as [python3](https://www.python.org/downloads/) configs set up!

To build this package in the base directory first call<br>

```
    source /opt/ros/humble/setup.bash
```
<br>
Then navigate towards the root of the workspace and build the project:<br>

```
    cd /path/to/workspace
    colcon build --symlink-install
```
<br>
That is, if everything was working which it isn't yet- so right now you'll have to use
the packages selection to make sure that it builds sucessfully, so alternativley run:<br>

```
    colcon build --packages-select arp_msgs arp_deployment_gp20 pcd_processing xml_processing reach reach_ros2 --symlink-install
```

<br>
Once the package has finished building make sure to source the install

```
    source ./install/setup.bash 
```
<br>
Update: Current sucessfull builds are arp_msgs, pcd_processing, and xml_processing - > key note with the pcd_processing though you'll have to have python configured for the compiler you're using!

### Executables

[**Main launch**]()
- arp_reach_config.launch.py (Not made yet--- [issue #24](https://github.com/natalieCloud/arp_reach/issues/24) TODO!)

[**arp_msgs**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_msgs) 
- None Y(^u^)Y

[**arp_deployment_gp20**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_deployment_gp20)
- None Y(^w^)Y
  
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
