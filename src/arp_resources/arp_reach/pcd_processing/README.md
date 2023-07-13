# pcd_processing README

Howdy! You've found the readme for the pcd_processing package! This package is one of many co-developed alongside another in order to provide ``Reach`` functionality to the [ar_paint](https://github.com/OSU-AIMS/augmented-reality-painting) project!

What pcd processing does is that is provides a service node that would read in the client ``PoseArray`` and write those poses to a file in the /tmp folder (Currently we are only setup to support Linux however 
an issue will be opened to make this package OS agnostic)

## Requirements to Build:

To sucessfully build this project one must have minimum [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed, as well as [python3](https://www.python.org/downloads/) configs set up!

Also, the package [arp_msgs](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_msgs) is nessesary for this package to build!

