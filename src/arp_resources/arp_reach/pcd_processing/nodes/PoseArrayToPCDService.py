#! /usr/bin/env

from arp_msgs.srv import FormatPosesToPCD
import pcd.pose_array_to_pcd as pcd

import rclpy
from rclpy.node import Node

class GeneratePCDService(Node):

    def __init__(self):
        super().__init__('pcd_processor_service')
        self.srv = self.create_service(FormatPosesToPCD, 'process_pcd', self.process_pcd_callback)

    def process_pcd_callback(self, request, response):
        self.get_logger().info("Called callback...\n")
        response.pcd_filepath = 'poseArr.pcd'
        pcd.write_file(response.pcd_filepath, request.waypoints)

        # file_out = open(response.pcd_file, "w")
        # size = len(request.waypoints.poses)

        # file_out.write("# .PCD v0.7 - Point Cloud Data file format\n")
        # file_out.write("VERSION 0.7\nFIELDS x y z normal_x normal_y normal_z curvature\n")
        # file_out.write("SIZE 4 4 4 4 4 4 4\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\n")
        # file_out.write("WIDTH " + str(size) + "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
        # file_out.write("POINTS " + str(size) + "\nDATA ascii\n")

        # for new_pose in request.waypoints.poses:
        #     file_out.write(str(new_pose.position.x) + " " + str(new_pose.position.y) + " ")
        #     file_out.write(str(new_pose.position.z) + " " + str(new_pose.orientation.x) + " ")
        #     file_out.write(str(new_pose.orientation.y) + " " + str(new_pose.orientation.z))
        #     file_out.write(" " + str(new_pose.orientation.w) + "\n")


        return response
    
def main(args=None):
    rclpy.init()
    print("Hello")
    pcd_processor_service = GeneratePCDService()

    rclpy.spin(pcd_processor_service)

    pcd_processor_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

