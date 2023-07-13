#! /usr/bin/env

import sys

from arp_msgs.srv import FormatPosesToPCD

from geometry_msgs.msg import PoseArray, Pose
import numpy as np

import rclpy
from rclpy.node import Node

class GeneratePCDClient(Node):

    def __init__(self):
        super().__init__('pcd_processor_client')
        self.cli = self.create_client(FormatPosesToPCD, 'process_pcd')
        self.get_logger().debug('test statement')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again...')
        self.req = FormatPosesToPCD.Request()

    def send_request(self, waypoints):
        self.req = waypoints
        print("Request sent")
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main(args=None):
    rclpy.init()
    print("Hello")
    print(args)
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

    response = pcd_processor.send_request(req)
    print("Request recieved")
    pcd_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        