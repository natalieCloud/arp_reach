#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com

import os

from arp_msgs.srv import FormatPosesToPCD

import file_makers.pose_array_to_pcd as pcd
import file_makers.yaml_file_maker as yaml

import rclpy
from rclpy.node import Node

class GeneratePCDService(Node):
    """
    This class represents a type of service node from the FormatPosesToPCD.srv found in the arp_msgs package!
    It takes in a request from the client which is comprised of a PoseArray, and then tries to run that 
    PoseArray into the pcd processor, and then the resulting pcd filepath into the yaml file creator!
    It then returns the yaml filepath and a boolean indicating sucess to the client, indicating whether
    or not to turn to phase 2 of the node series! :)
    
    ...

    Methods
    -------
    __init__(self):
        Initilizes the node and sends a request to its callback function.

    process_pcd_callback(self, request, response):
        Grabs the client's waypoints and runs them through the pcd_file_maker as well as the yaml_file_maker!
        Returns the yaml filepath as well as a boolean indicating sucess through the response!
    """

    def __init__(self):
        """
        Constructs all the nessesary attributes for the Node object
        """
        super().__init__('pcd_processor_service')
        self.srv = self.create_service(FormatPosesToPCD, 'process_pcd', self.process_pcd_callback)

    def process_pcd_callback(self, request, response):
        """
        Calls the write file function based on the poseArray given by the client node!
        """
        self.get_logger().info("Called callback...\n")

        pcd_filepath = str(os.path.join(os.path.dirname(os.getcwd()), "install", "arp_reach", "share", "arp_reach", "poseArray.pcd"))
        yaml_filepath = str(os.path.join(os.path.dirname(os.getcwd()), "install", "arp_reach", "share", "arp_reach", "study_config.yaml"))

        response.yaml.yaml_filepath = yaml_filepath

        try:
            pcd.write_file(pcd_filepath, request.waypoints)
            yaml.write_yaml(response.yaml.yaml_filepath, 'package://arp_reach/poseArray.pcd') #Look into package pathing resource packs
        except (ValueError, RuntimeError):
            response.sucess = False
        
        response.sucess = True

        return response

def main():
    """
    Main body for the node! Takes in no arguments, constructs a service node,
    and starts to run that node! Waits for input from the client to process the
    points into a file!
    """
    rclpy.init()
    pcd_processor_service = GeneratePCDService()

    rclpy.spin(pcd_processor_service)

    pcd_processor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
