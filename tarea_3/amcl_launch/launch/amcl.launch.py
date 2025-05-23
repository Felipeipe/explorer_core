import os
from launch.frontend.parser import parse_if_substitutions
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction

def generate_launch_description():

    # Package name
    package_name='amcl_launch'

    # Launch configurations

    map_yaml = os.path.join(get_package_share_directory(package_name),'map','mapa.yaml')

    # Launch the amcl node
    amcl_params = os.path.join(get_package_share_directory(package_name),'config','amcl.yaml')
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[amcl_params],
        
    )

    map_server_params = os.path.join(get_package_share_directory(package_name),'config','map_server.yaml')
    map_server = Node(
        name='map_server',
        package='nav2_map_server',
        executable='map_server',
        parameters=[
            map_server_params,
            {'yaml_filename': map_yaml},
            {'use_sim_time': True},
        ],
        respawn=True,
        respawn_delay=0.5,
    )

    
    # Launch them all!
    return LaunchDescription([
        # Declare launch arguments

        # Launch the nodes
        amcl_node,
        map_server,
    ])
