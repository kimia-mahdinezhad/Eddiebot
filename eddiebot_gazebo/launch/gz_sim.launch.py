import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('world', default_value='empty',
                          description='Gazebo World'),
    DeclareLaunchArgument('use_sim_time', default_value='false',
                          description='Use simulation (Gazebo) clock if true'),
]


def generate_launch_description():

    # Directories
    pkg_ros_gz_sim = get_package_share_directory(
        'ros_gz_sim')
    pkg_eddiebot_gazebo = get_package_share_directory(
        'eddiebot_gazebo')
    pkg_eddiebot_description = get_package_share_directory(
        'eddiebot_description')

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_eddiebot_gazebo, 'worlds'), ':' +
            str(Path(pkg_eddiebot_gazebo).parent.resolve()), ':' +
            str(Path(pkg_eddiebot_description).parent.resolve())
            ]
        )

    # Paths
    gz_sim_launch = PathJoinSubstitution(
        [pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', [LaunchConfiguration('world'), '.sdf'])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[gz.msgs.Clock'
                        ])

    # Create launch description and add actions
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    return ld
