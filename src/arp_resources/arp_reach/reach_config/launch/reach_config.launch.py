from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    launch_description = LaunchDescription()

    study_node_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('reach_config'), 
                     'launch/reach_study_node.launch.py')
        )
    )

    launch_description.add_action(study_node_file)

    return launch_description