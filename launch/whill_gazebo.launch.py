from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Gazebo 起動
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-r', 'empty.sdf'],
            output='screen'),

        # WHILL モデルを spawn
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', '/PATH/TO/whill_description/urdf/whill.urdf',
                '-name', 'whill'
            ],
            output='screen'),
    ])

