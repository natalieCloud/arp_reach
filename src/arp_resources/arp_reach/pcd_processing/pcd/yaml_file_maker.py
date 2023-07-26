#! /usr/bin/env

# Author: Natalie Chmura
# Maintainer: Natalie Chmura email:ntchmura@gmail.com

import yaml

def write_yaml(out_file_name, pcd_filename):
    """
    This takes the information from poseArray and parses it into a yaml file that will be
    used as a parameter in the reach study!

    Parameters
    ----------
        out_file_name : str
            The file that will be written to!
        pcd_filename: str
            The file that needs to be included as the pcd file!
    """
    data = {
        'optimization':{
            'radius':'0.2',
            'max_steps':'10', 
            'step_improvment_threshold':'0.01'},
        'ik_solver':{
            'name':'MoveItIKSolver',
            'distance_threshold':'0.0',
            'planning_group':'manipulator', 
            'touch_links': '[]'},
        'evaluator':{
            'name':'MultiplicativeEvaluator', 
            'plugins': 
                [{'name':'ManipulabilityMoveIt',
                'planning_group':'manipulator'},
                {'name':'DistancePenaltyMoveIt',
                'planning_group':'manipulator',
                'distance_threshold':'0.025',
                'exponent':'2',
                'touch_links':'[]'},],
                },
        'display':{
            'name':'ROSDisplay',
            'kinematic_base_frame':'base_link',
            'marker_scale':'0.05',
            'use_full_color_range':'true',
        },
        'target_pose_generator':{
            'name':'PointCloudTargetPoseGenerator',
            'pcd_file':pcd_filename,
        },
        'logger':{
            'name':'BoostProgressConsoleLogger',
        },
    }

    with open(out_file_name, 'w') as file:
        yaml.dump(data, file)


