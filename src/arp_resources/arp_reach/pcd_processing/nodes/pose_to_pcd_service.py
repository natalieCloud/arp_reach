#! /usr/bin/env

from arp_msgs.srv import FormatPosesToPCD
import pcd.pose_array_to_pcd as pcd

import rclpy
from rclpy.node import Node

##
# PCD Processor Node
##
class ProcessPosePCD(Node):
    """
        A class which crates a service for taking in a group of sample poses representing
        a path or a part (All represented by a PoseArray[]) and generates a PCD file from
        those poses.
    """

    def __init__(self):
        super().__init__('pcd_processor_service')
        self.srv = self.create_service(FormatPosesToPCD, 'process_pcd', self.process_pcd_callback)

    def process_pcd_callback(self, request, response):
        response.pcd_filepath = '~/tmp/poseArr.pcd'
        pcd.write_file(response.pcd_filepath, request.waypoints)

        return response

    
def main(args=None):
    rclpy.init()

    pcd_processor_service = ProcessPosePCD()

    rclpy.spin(pcd_processor_service)

    pcd_processor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()