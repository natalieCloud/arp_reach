#! /usr/bin/env

import sys

from arp_msgs.srv import FormatPosesToPCD

import rclpy
from rclpy.node import Node

class GeneratePCDClient(Node):

    def __init__(self):
        super().__init__('pcd_processor_client')
        self.cli = self.create_client(FormatPosesToPCD, 'process_pcd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not avaliable, waiting again...')
        self.req = FormatPosesToPCD.Request()

    def send_request(self, waypoints):
        self.req.waypoints = waypoints
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
def main():
    rclpy.init()
    pcd_processor = GeneratePCDClient()
    result = pcd_processor.send_request(sys.argv[1])
    
    pcd_processor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        