import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction  
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def check_param_file(context, *args, **kwargs):
    param_dir = LaunchConfiguration('param_dir').perform(context)
    param_filename = LaunchConfiguration('param_filename').perform(context)
    param_path = os.path.join(param_dir, param_filename)
    if not os.path.isfile(param_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {param_path}")
    return []

def generate_launch_description():
    pcd_map_server_dir = get_package_share_directory('pcd_map_server')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    param_filename = LaunchConfiguration('param_filename')
    param_dir = LaunchConfiguration('param_dir')
    default_param_dir = PathJoinSubstitution([pcd_map_server_dir, 'param'])
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (sim time)'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS namespace'
    )

    declare_param_filename_cmd = DeclareLaunchArgument(
        'param_filename', default_value='param.yaml',
        description='Param YAML filename inside the param/ folder'
    )

    declare_param_dir_cmd = DeclareLaunchArgument(
        'param_dir',
        default_value=default_param_dir,
        description='Absolute path to the parameter YAML file'
    )

    pcd_map_server_node = Node(
            package='pcd_map_server',
            executable='pcd_map_server_node',
            namespace=namespace,
            output='screen',
            parameters=[
            {'use_sim_time': use_sim_time},
            PathJoinSubstitution([param_dir, param_filename]),
            ],
        )
    
    check_params = OpaqueFunction(function=check_param_file)

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_param_filename_cmd)
    ld.add_action(declare_param_dir_cmd)
    ld.add_action(check_params)
    ld.add_action(pcd_map_server_node)
    
    return ld