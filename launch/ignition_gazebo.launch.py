# launch/ignition_gazebo.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    declare_world = DeclareLaunchArgument('world', default_value='')
    world = LaunchConfiguration('world')

    gz_cmd = ['ros2', 'launch', 'ros_gz_sim', 'gz_sim.launch.py']
    gz_cmd += ['world:=', LaunchConfiguration('world')]

    gz_proc = ExecuteProcess(cmd=gz_cmd, output='screen')

    return LaunchDescription([ declare_world, gz_proc ])