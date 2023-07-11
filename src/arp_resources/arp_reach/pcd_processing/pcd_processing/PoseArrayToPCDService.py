from arp_msgs.srv import FormatPosesToPCD
from pcd.generate import pose_array_to_pcd

import rclpy
from rclpy.node import Node

class GeneratePCDService(Node):

    def __init__(self):
        super.__init__('pcd_processor_service')
        self.srv = self.create_service(FormatPosesToPCD, 'process_pcd', self.process_pcd_callback)

    def process_pcd_callback(self, request, response):
        response.pcd_file = '/tmp/poseArr.pcd'
        pose_array_to_pcd.write_file('/tmp/poseArr.pcd', request.waypoints)
        return response
    
def main():
    rclpy.init()

    pcd_processor_service = GeneratePCDService()

    rclpy.spin(pcd_processor_service)

    rclpy.shutdown()

if __name__ == '__main__':
    main()

