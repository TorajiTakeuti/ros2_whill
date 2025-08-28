import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def get_robot_description(context):
    pkg_path = FindPackageShare('whill_description').find('whill_description')
    xacro_file = os.path.join(pkg_path, 'robot_top.urdf.xacro')
    
    robot_description = Command([FindExecutable(name='xacro'), ' ', xacro_file])
    
    return [
        DeclareLaunchArgument('entity', default_value='my_robot'),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_sim', 'create', '-string', robot_description, '--name', LaunchConfiguration('entity')],
            output='screen'
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=get_robot_description)
    ])