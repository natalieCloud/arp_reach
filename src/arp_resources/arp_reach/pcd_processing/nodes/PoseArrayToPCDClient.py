#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com

from arp_msgs.srv import FormatPosesToPCD

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray, Pose
import numpy as np

class GeneratePCDClient(Node):
    """
    A class to represent a Node object of type client that takes in a poseArray and 
    generates a pcd file from a service!
    
    ...

    Methods
    -------
    __init__(self):
        Initilizes the node and sends a request to the service should the service be active
        else, waits and tries again.

    send_request(self, waypoints):
        Assigns the waypoints to itself and spins until complete (May not be sucessful) returns
        the result!
    """

    def __init__(self):
        """
        Constructs all the nessesary attributes for the Node object
        """
        super().__init__('pcd_processor_client')
        self.cli = self.create_client(FormatPosesToPCD, 'process_pcd')
        self.get_logger().debug('test statement')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again...')
        self.req = FormatPosesToPCD.Request()

    def send_request(self, waypoints):
        """
        Sends the node's request to the service! May not return anything if the service 
        is not active!
        """
        self.req = waypoints
        #print("Request sent")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    """
    Main body for the node! Takes in no arguments, constructs a poseArray(to simulate the message
    that will be passed in) contructs a request from that, and then initilizes a new client node
    from that poseArray!
    """
    rclpy.init()
    #print("Hello- this is the client")
    #print(args)
    pcd_processor = GeneratePCDClient()

    req = FormatPosesToPCD.Request()
    m = PoseArray()
    m.header.frame_id = 'world'
    for i in np.arange(0.0, 2.0, 0.5):
        p = Pose()
        p.orientation.x = 0.0
        p.orientation.y = 0.0
        p.orientation.z = 0.0
        p.orientation.w = 1.0
        p.position.x = i
        p.position.y = 0.1
        p.position.z = 2.0
        m.poses.append(p)
    req.waypoints = m

    pcd_processor.send_request(req)
    #print("Request recieved")
    pcd_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        