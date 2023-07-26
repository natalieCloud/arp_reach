# arp_msgs README

This package contains a subset of messages that will be used in the repos [PCD_PROCESSING](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing),
[Reach_Config](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/reach_config),
and [XML_PROCESSING](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing)

They will be merged into the `arg_msgs` at [augmented_reality_painting](https://github.com/OSU-AIMS/augmented-reality-painting)

## Msgs

**PCDFile**

Stores the filepath of a pcd file

**XMLFILE**

Stores the filepath of an xml file

**YAMLFILE**

Stores the filepath of a yaml file

## Srv

**CheckGridsetReachability**

Passes in the filepath of a pcd file and runs a reach study on those points- returns the xml file containing the reach study results

**FormatPosesFromXML**

Takes in the reach study result xml file and processes the file into the `PoseArray` and the reach score array

**FormatPosesToPCD**

Takes in the `PoseArray` and generates a YAML file from those poses- returns the filepath to those poses.
