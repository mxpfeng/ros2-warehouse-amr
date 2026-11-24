"""One-command full stack: Gazebo + robot + truth-localized Nav2 + RViz.

    ros2 launch warehouse_amr_description bringup_sim.launch.py

Add ``gui:=false`` for headless CI, and ``rviz:=false`` when driving the demo
from the mission node only.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('warehouse_amr_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    #map_yaml = LaunchConfiguration('map')
    map_default = os.path.join(pkg_share, 'maps', 'warehouse.yaml')
    params_default = os.path.join(pkg_share, 'config', 'nav2_params.yaml')


    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'launch_sim.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world,
            'gui': gui,
        }.items(),
    )

    # Give Gazebo and the robot model time to start before truth localization
    # and Nav2 begin.
    nav = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    #'map': map_yaml,
                    'map': map_default,            # ← 直接传字符串路径，不通过 LaunchConfiguration
                    'params_file': params_default,  # ← 补上 params_file
                    'autostart': 'True',
                }.items(),
            )
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(pkg_share, 'config', 'main.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_share, 'worlds', 'warehouse.world')),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz2'),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(pkg_share, 'maps', 'warehouse.yaml')),
        sim,
        nav,
        rviz_node,
    ])
