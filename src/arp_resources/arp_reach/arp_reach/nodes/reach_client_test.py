#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com
import os

from arp_msgs.srv import RunReachStudy

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class ArpReachClient(Node):
    """
    A class to represent a Node object of type client that takes in a yaml filepath, reach directory, and name
    to run the reach study through!

    ...
    Methods
    -------
    __init__(self):
        Initilizes the node and send a request to the service should the service be active. Else waits and tries again!

    send_request(self):
        Creates and assigns the request variables throughb the use of os.join.path and then spins the node until complete!
    """

    def __init__(self):
        """
        Constructs all of the nessesary attributes for the Node object
        """
        super().__init__('arp_reach_client')
        self.cli = self.create_client(RunReachStudy, 'arp_reach')
        self.get_logger().debug('Client created')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not avaliable, waiting again.....')
        self.req = RunReachStudy.Request()

    def send_request(self, yaml_filepath, config_name, results_dir):
        """
        Sends the node's request to the service! Assigns the filepaths, directories, and config name to the
        request param! May not return anything if the service is not active! 
        """
        self.req.yaml_filepath = yaml_filepath
        self.req.config_name = config_name
        self.req.results_dir = results_dir

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    """
    Main body for the node! Takes in no arguments, constructs the filpaths, constructs a request from those filepaths,
    and then initilizes a new client node from that pose!
    """
    rclpy.init()

    arp_reach_client = ArpReachClient()

    yaml_file = str(os.path.join(os.path.dirname(os.getcwd()), "arp_reach", "src", "arp_resources", "arp_reach", "arp_reach", "config", "study_config.yaml"))
    config_name = 'study_config'
    results_dir = str(os.path.join(os.path.dirname(os.getcwd()), "install", "reach_ros", "share", "reach_ros"))

    response = arp_reach_client.send_request(yaml_file, config_name, results_dir)
    arp_reach_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
