# arp_msgs README

This package contains a subset of messages that are nessesary for the nodes created and run in the repos [PCD_Processing](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing),
[ARP_Reach](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/arp_reach),
and [XML_Processing](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing)

They will be merged into the `arg_msgs` at [augmented_reality_painting](https://github.com/OSU-AIMS/augmented-reality-painting)

## Msgs

**PCDFile:**

Stores the filepath of a pcd file

**XMLFILE:**

Stores the filepath of an xml file

**YAMLFILE:**

Stores the filepath of a yaml file

## Srv

**CheckGridsetReachability:**

Passes in the filepath of a pcd file and runs a reach study on those points- returns the xml file containing the reach study results (May become deprecated soon)

**FormatPosesFromXML:**

Takes in the reach study result xml file and a PoseArray of waypoints to get the scores for, and processes the file into the `PoseArray` and the reach score array, as well as a boolean indicating if the process was sucesful or not

**FormatPosesToPCD:**

Takes in the `PoseArray` and generates a YAML file from those poses- returns the filepath to those poses, as well as a boolean indicating if the formatting to pcd was a sucess.

**Run Reach Study:**

Takes in the yaml configuration file, the configuration name, directory of where the results should be written to, and a boolean signaling if the study can be run or not, and then returns the full path to where the reach results are kept, a boolean indicating if the study running was sucessful, and a message giving that the study was sucesssful, or an error if applicanble!
