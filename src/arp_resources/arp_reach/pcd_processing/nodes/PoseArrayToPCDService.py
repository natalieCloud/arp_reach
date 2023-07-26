#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com

import os

from arp_msgs.srv import FormatPosesToPCD
import pcd.pose_array_to_pcd as pcd
import pcd.yaml_file_maker as yaml

import rclpy
from rclpy.node import Node

class GeneratePCDService(Node):
    """
    A class to represent a Node object of type service that takes in a poseArray and 
    generates a pcd file from the file writer!
    
    ...

    Methods
    -------
    __init__(self):
        Initilizes the node and sends a request to the its function.

    process_pcd_callback(self, waypoints):
        Assigns the waypoints to itself and spins until complete (May not be sucessful) returns
        the result!
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
        home_dir = os.path.expanduser('~')
        pcd_filepath = home_dir + '/../../tmp/poseArr.pcd'
        response.yaml_filepath = home_dir + '/../../tmp/tempYaml.yaml'
        pcd.write_file(pcd_filepath, request.waypoints)
        yaml.write_yaml(response.yaml_filepath, pcd_filepath)

        return response

def main():
    """
    Main body for the node! Takes in no arguments, constructs a service node,
    and starts to run that node! Waits for input from the client to process the
    points into a file!
    """
    rclpy.init()
    #print("Hello- this is the service")
    pcd_processor_service = GeneratePCDService()

    rclpy.spin(pcd_processor_service)

    pcd_processor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

