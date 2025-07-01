import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    bento_drive = Node(
        package='evotrainer_drive',
        executable='evotrainer_drive_node',
        name='evotrainer_drive_node',
        parameters=[ PathJoinSubstitution([ FindPackageShare('evotrainer_drive'), 'parameter', 'evotrainer.yaml' ]) ],
        namespace=EnvironmentVariable( 'EDU_ROBOT_NAMESPACE', default_value="bento" ),
        output='screen'
    )

    return LaunchDescription([
        bento_drive
    ])
