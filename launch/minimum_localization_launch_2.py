import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.substitutions import TextSubstitution

def generate_launch_description():
    # Hardcoded values
    bringup_dir = get_package_share_directory('f112th_sim_2501_bravo')
    map_yaml_file = os.path.join(bringup_dir, 'maps', 'your_map.yaml')
    params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    use_sim_time = TextSubstitution(text='true')
    autostart = TextSubstitution(text='true')
    log_level = TextSubstitution(text='info')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    lifecycle_nodes = ['map_server', 'amcl']

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': TextSubstitution(text=map_yaml_file),
            },
            convert_types=True),
        allow_substs=True
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': lifecycle_nodes
            }],
            arguments=['--ros-args', '--log-level', log_level]
        )
    ])

