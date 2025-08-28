# launch/robot_description.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_gazebo = DeclareLaunchArgument('gazebo', default_value='ignition')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gazebo = LaunchConfiguration('gazebo')

    xacro_file = PathJoinSubstitution([FindPackageShare('whill_description'), 'robot_top.urdf.xacro'])
    robot_description_command = Command(['xacro ', xacro_file, ' ', 'gazebo:=', gazebo])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_command, 'use_sim_time': use_sim_time}]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([ declare_use_sim_time, declare_gazebo, rsp_node, jsp_node ])