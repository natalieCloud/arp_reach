# arp_reach README

Greetings! You've found the readme for the arp_reach package! This package is one of the many codeveloped so aupport the addition of `Reach`
functionality from [ROS_Industrial](https://github.com/ros-industrial/reach) to the [arpaint](https://github.com/OSU-AIMS/augmented-reality-painting)
project!

What the arp_reach package does is create a node that is launched in a similar manner to `Reach_ROS`'s reach_study_node, 
however this package sets the node up as a service rather than a free spinning node. The service is set up to take in four request
parameters by the pair client node, namley a filepath to the `YAML` configuration file, the name of the configuration file, the name of the result directory,
and finally a boolean indicating that the reach study is good to be run (Taken from the boolean response from the [pcd_processing service](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing)
! Once the reach study has been run the service returns the full filpath to the reach result directory, as 
well as a boolean indicating that the study has been run sucessfully and the next phase is ready to be run. Referring to of course 
the [xml_processing service](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing)!

![arp_reach](https://github.com/natalieCloud/arp_reach/assets/123828141/9dcf70fc-805c-4103-9333-d92bd09dba7d)

## Requirements to Build:
To sucessfully build this project one must have minimum [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed, as well as [python3](https://www.python.org/downloads/) configs set up!

Also, the package [arp_msgs](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_msgs), and the submodules
[reach](https://github.com/ros-industrial/reach), 
[reach_ros2](https://github.com/ros-industrial/reach_ros2), 
and [boost_interfaces]( https://github.com/tesseract-robotics/boost_plugin_loader) 
are nessesary for this package to build!

## Nodes

### [ReachClient.cpp](https://github.com/natalieCloud/arp_reach/blob/main/src/arp_resources/arp_reach/arp_reach/nodes/reachClient.cpp)
This file runs a reach_study based node that functions as a client calling the launch file launched service node, as defined
by [RunReachStudy.srv](https://github.com/natalieCloud/arp_reach/blob/main/src/arp_resources/arp_msgs/srv/RunReachStudy.srv)!
The client's request is comprised of a  `config_file`, `config_name`, `reach results directory`, and a `boolean`
that indicates when the study can be run, along with the other parameters given by the launch file (which is how this
node is spun in the first place as opposed to the other ones that can be launched singularly, the launch parameters
help with the `reach_ros` interfaces that make up the reach study!) It is then sent to the service node, and recives 
A filepath to the reach results file, as a well as a boolean indicating that the reach study ran sucessfully!

### [ReachService.cpp](https://github.com/natalieCloud/arp_reach/blob/main/src/arp_resources/arp_reach/arp_reach/nodes/reachStudyService.cpp)
This file runs a `reach_study` based node that functions as a service rather than a free spinning node as defined
by [RunReachStudy.srv](https://github.com/natalieCloud/arp_reach/blob/main/src/arp_resources/arp_msgs/srv/RunReachStudy.srv)! 
The node takes in the client's request with the `config_file`, `config_name`, `reach results directory`, and a `boolean` 
that indicates when the study can be run, along with the other parameters given by the launch file (which is how this
node is spun in the first place as opposed to the other ones that can be launched singularly, the launch parameters
help with the `reach_ros` interfaces that make up the reach study!)

## Support Files:

See the refferenced submodules above to see the files used to run the reach study!
