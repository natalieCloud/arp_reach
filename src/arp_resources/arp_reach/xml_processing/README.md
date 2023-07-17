# xml_processing README

Hello there! This is the readmne for the xml_processing package! This package is one of the many deleoped alongside others in order to provide `Reach` functionality to the [ar_paint](https://github.com/OSU-AIMS/augmented-reality-painting) project!

What xml_processing does is that it provides a service node that would read in the client's `Pose Array` and `Reach DB file` and parses through the database (XML) file, and returns the original `Pose Array` with a corresponding array of `reach scores`! Both of these files are currently stored in the tmp folder (which is unfortunatley not OS agnostic :(  ...yet- see [issue #13](https://github.com/natalieCloud/arp_reach/issues/13))

## Requirements to Build:

To sucessfully build this project one must have minimum [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) installed, as well as [python3](https://www.python.org/downloads/) configs set up!

Also, the package [arp_msgs](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_msgs) is nessesary for this package to build!

## Important Features!

A key part of this code, and something we had to offset with rounding (rather than dismantaling reach as a whole (as of right not at least- there is possible cause for that down the road) However as of right now reach runs the study - taking in a pose represented by a position (xyz) and qwuaternion (wxyz) and converts those data forms to type Isometry 3D! As a result, when reading the reach data and making the conversion back to the pose array position-quaternion form, the data- originally in Float64 form that goes out to 6-9 decimal plces, now goes out at least 16! As a result it was deemed nessesary to round down the results (In a sense of registering the keys and poses) to return the appropriate reach score per pose! This rounding occurs at two places:
1) array_transform - When the result data is added to the map it is done so with the rounded values rather than the ones calculated during conversion of isometry to quaternion!
2) result_parsing - When the poseArray data is getting passed to the map the values are rounded down to match the values of the keys!

Due to the original sizing of the pose and typical data in that range, it was deemed appropriate to round to a scale of 1/10000000 - or in practical (reality measurements) scale; A nanometer! Due to the nature of the points provided by and used by our path planning algorithm, it was deemed that this was an appropriate range to round to! 
