#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

PACKAGE_NAME = 'grupa_4'

def generate_launch_description():
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    pkg_share = FindPackageShare(package='grupa_4').find('grupa_4')
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    world_file_name = 'balls.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)
    print(world_path)
    gazebo_models_path = os.path.join(pkg_share, 'models')
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path
    print(gazebo_models_path)

    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    declare_simulator_cmd = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load')

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py'),
        ),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world}.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

    tb3_urdf_path = os.path.join(
        gazebo_models_path,
        'tb3_burger_camera.urdf'
    )

    tb3_x_pose = LaunchConfiguration('tb3_x_pose', default='0.0')
    tb3_y_pose = LaunchConfiguration('tb3_y_pose', default='0.0')

    spawn_tb3 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'tb3_burger_camera',
            '-file', tb3_urdf_path,
            '-robot_namespace', 'robot',
            '-x', tb3_x_pose,
            '-y', tb3_y_pose,
            '-z', '0.01'
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        arguments=[tb3_urdf_path]
    )

    ld = LaunchDescription()


    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    #ld.add_action(robot_state_publisher_cmd)
    #ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_tb3)
    return ld
