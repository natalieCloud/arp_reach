# reach_launch README

Hello There! You've stumbled upon the readme for the reach_launch package! This package's primary funtion is to launch all of the nodes found in the required for creating the config files for, launching the `reach_study`, and parsing its output!

This package comprises of six nodes:

1. The client and service for [**pcd_processing**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/pcd_processing)

2. The launch service node and client for [**reach_config**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/reach_config)

3. The client and service for [**xml_processing**](https://github.com/natalieCloud/arp_reach/tree/main/src/arp_resources/arp_reach/xml_processing)

![reach_launch](https://github.com/natalieCloud/arp_reach/assets/123828141/f3187953-c1d1-459b-acba-2e844b2dff67)

## Launch Files

### [Launch Reach_Services](https://github.com/natalieCloud/arp_reach/blob/documentation/issue-2/update_readme/src/arp_resources/arp_reach/launch/reach_launch.xml)
The service nodes have been sperated out to be simutanueously launched and spin continuously until the client nodes call on them!

### [Launch_Reach_Clients]()
The client nodes, while set up to launch do so in seqential fashion, much like how the state machine will construct 'client-like' behavior by calling on the services sequentially. The boolean signal and sucess in the request and response are included to signal that 'next_step' is allowed!

