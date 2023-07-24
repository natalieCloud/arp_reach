#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com

from geometry_msgs.msg import Pose, PoseArray

###
 # @brief A python file that processes an input of type poseArray and turns it into a .pcd 
 # file that can be used in the reach study
 ##

##
# @mainpage PoseArray to PCD Converter
#
# @section description_main Description
# A Python file that processes a PoseArray generated by the current "path-planning"
# algorithm into a pcd format that is able to be processed by ros-industrial reach!
#
# @section notes_main Example
# INPUT:
# {header:
#   {frame_id: 'world'}, poses: [
#       {
#           position: {
#               x: 1.0
#               y: 0.0
#               z: 0.0},
#           orientation: {
#               x: 0.0
#               y: 0.0
#               z: 0.0
#               w: 0.0}
#       }, {
#           position: {
#               x: 1.1
#               y: 0.0
#               z: 0.0},
#           orientation: {
#               x: 0.0
#               y: 0.0
#               z: 0.0
#               w: 0.0}}]}
# OUTPUT:
# # .PCD v0.7 - Point Cloud Data file format
# VERSION 0.7
# FIELDS x y z normal_x normal_y normal_z curvature
# SIZE 4 4 4 4 4 4 4
# TYPE F F F F F F F
# COUNT 1 1 1 1 1 1 1
# WIDTH 2 // The number of rows of the original file
# HEIGHT 1
# VIEWPOINT 0 0 0 1 0 0 0
# POINTS 2 // Same as the width above
# DATA ascii
# 1.0 0.0 0.0 0.0 0.0 0.0 1.0
# 1.1 0.0 0.0 0.0 0.0 0.0 1.0
##

# FUNCTIONS:

    ## write_file
    #
    # This takes the information from poseArray and parses it into a .pcd file that will allow us 
    # to run the reach study
    #
    # @param outFile: The file that will be written to (Path wlll be determined by launch param?
    # or maybe just be constant idk yet!
    #
    # @param poseArr: The poseArray that all the information will be taken from!
    #
def write_file(out_file_name, pose_arr):
    """
    This takes the information from poseArray and parses it into a .pcd file that will allow us 
    to run the reach study

    Parameters
    ----------
        outFile : str
            The file that will be written to (Path wlll be determined by launch param?
            or maybe just be constant idk yet!
        poseArr : PoseArray
            The poseArray that all the information will be taken from!
    """
    file_out = open(out_file_name, "w")
    size = len(pose_arr.poses)

    file_out.write("# .PCD v0.7 - Point Cloud Data file format\n")
    file_out.write("VERSION 0.7\nFIELDS x y z normal_x normal_y normal_z curvature\n")
    file_out.write("SIZE 4 4 4 4 4 4 4\nTYPE F F F F F F F\nCOUNT 1 1 1 1 1 1 1\n")
    file_out.write("WIDTH " + str(size) + "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
    file_out.write("POINTS " + str(size) + "\nDATA ascii\n")

    for new_pose in pose_arr.poses:
        file_out.write(str(new_pose.position.x) + " " + str(new_pose.position.y) + " ")
        file_out.write(str(new_pose.position.z) + " " + str(new_pose.orientation.x) + " ")
        file_out.write(str(new_pose.orientation.y) + " " + str(new_pose.orientation.z))
        file_out.write(" " + str(new_pose.orientation.w) + "\n")

    return out_file_name

def read_poses(in_file_name):
    """
        This takes the information in from a PCD file and parses it into a poseArray that will
        allow for ease of testing

        Parameters
        ----------
            in_file_name : str
                The file that will be read from (Path will be determined by user)
    """
    poses = PoseArray()

    file_in = open(in_file_name, "r")
    file_in.readline() # PCD v0.7 etc
    file_in.readline() # Version 0.7
    file_in.readline() # Fields
    file_in.readline() # Size
    file_in.readline() # Type
    file_in.readline() # Count
    file_in.readline() # Width
    file_in.readline() # Height
    file_in.readline() # Viewpoint
    file_in.readline() # Points
    file_in.readline() # Data

    while True:
        line = file_in.readline()
        if not line:
            break
        line.strip('\n')
        nums = [float(x) for x in line.split()]
        pose = Pose()
        pose.orientation.w = nums[3]
        pose.orientation.x = nums[4]
        pose.orientation.y = nums[5]
        pose.orientation.z = nums[6]
        pose.position.x = nums[0]
        pose.position.y = nums[1]
        pose.position.z = nums[2]
        poses.append(pose)

    file_in.close()

    return poses
