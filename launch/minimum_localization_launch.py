import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
from launch_ros.descriptions import ParameterFile
from launch.substitutions import TextSubstitution


def generate_launch_description():
    # Hardcoded values

    use_sim_time = TextSubstitution(text='true')
    autostart = TextSubstitution(text='true')
    log_level = TextSubstitution(text='info')
    namespace = TextSubstitution(text='')
    use_respawn = TextSubstitution(text='false')

    map_file = 'walls_one.yaml'
    nav2_param_file = 'nav2_params.yaml'
    bringup_dir = get_package_share_directory('f112th_sim_2501_bravo')
    map_yaml_file = os.path.join(bringup_dir, 'maps', map_file)
    print(map_yaml_file)
    params_file = os.path.join(bringup_dir, 'config', nav2_param_file)

    remappings = [('/tf', 'tf'), 
                  ('/tf_static', 'tf_static')]
    lifecycle_nodes = ['map_server', 'amcl']

   # Create our own temporary YAML files that include substitutions
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml_file,
            },
            convert_types=True),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

    # Declare the launch arguments
    nav2_map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        ),
    # Create the launch description and populate
    nav2_acml_node = Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            respawn=use_respawn,
            parameters=[configured_params],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings
        ),
    
    nav2_lifecycle_manager_node = Node(
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


    return LaunchDescription([stdout_linebuf_envvar,
                              nav2_map_server_node,
                              nav2_acml_node,
                              nav2_lifecycle_manager_node  
    ])



