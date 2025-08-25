#!/usr/bin/env python3
"""
親ラウンチ: 既存の子ラウンチ (ignition_gazebo, robot_description, controllers, spawn_robot) をまとめて起動する
保存場所:
  ~/whill_ws/src/ros2_whill/whill_description/launch/parent_bringup.launch.py
実行:
  source ~/whill_ws/install/setup.bash
  ros2 launch whill_description parent_bringup.launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('whill_description')

    # 各子ラウンチファイルへのパス
    ignition_launch = os.path.join(pkg, 'launch', 'ignition_gazebo.launch.py')
    robot_desc_launch = os.path.join(pkg, 'launch', 'robot_description.launch.py')
    controllers_launch = os.path.join(pkg, 'launch', 'controllers.launch.py')
    spawn_launch = os.path.join(pkg, 'launch', 'spawn_robot.launch.py')

    # Include each child launch
    include_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ignition_launch)
    )
    include_robot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_desc_launch)
    )
    include_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(controllers_launch)
    )
    include_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch)
    )

    # 起動順序の例:
    #  1) Gazebo（ignition）を起動
    #  2) robot_description（URDF を robot_description トピックなどに流す）
    #  3) controllers（controller_manager の立ち上げ）
    #  4) spawn（ロボットを Gazebo に spawn）
    #
    # 必要なら順序を入れ替えたり、条件やパラメータを追加してください。
    return LaunchDescription([
        include_ign,
        include_robot_desc,
        include_controllers,
        include_spawn,
    ])
