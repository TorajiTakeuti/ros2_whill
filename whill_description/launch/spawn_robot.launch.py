# launch/spawn_robot.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    declare_entity_name = DeclareLaunchArgument('entity', default_value='my_robot')
    declare_namespace = DeclareLaunchArgument('namespace', default_value='')
    entity = LaunchConfiguration('entity')

    # xacro を評価して URDF を生成するコマンド（Substitution）
    urdf_xacro = PathJoinSubstitution(
        [FindPackageShare('whill_description'), 'urdf', 'robot_top.urdf.xacro']
    )
    robot_urdf_cmd = Command(['xacro ', urdf_xacro])

    # ros_gz_sim create に --stdin で xacro 出力を渡す
    spawn_cmd = ['ros2', 'run', 'ros_gz_sim', 'create', '--stdin', '--name', entity]

    spawn_proc = ExecuteProcess(
        cmd=spawn_cmd,
        stdin=robot_urdf_cmd,   # ここで xacro の出力を stdin に渡す
        output='screen'
    )

    return LaunchDescription([
        declare_entity_name,
        declare_namespace,
        spawn_proc
    ])