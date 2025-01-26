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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    pkg_turtlebot3_mogi = get_package_share_directory('turtlebot3_mogi')

    os.environ["GZ_SIM_RESOURCE_PATH"] += os.pathsep + pkg_turtlebot3_mogi + "/gazebo_models"

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    x_pose_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='x coordinate of spawned robot'
    )

    y_pose_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='y coordinate of spawned robot'
    )

    z_pose_arg = DeclareLaunchArgument(
        'z_pose', default_value='0.3',
        description='z coordinate of spawned robot'
    )

    yaw_angle_arg = DeclareLaunchArgument(
        'yaw_angle', default_value='0.0',
        description='yaw angle of spawned robot'
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='light_background.sdf',
        description='Name of the Gazebo world file to load'
    )

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Open RViz'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='turtlebot3_line_follower.rviz',
        description='RViz config file'
    )

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'models'))

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [PathJoinSubstitution([
            pkg_turtlebot3_mogi,
            'worlds',
            LaunchConfiguration('world')
        ]),
        #TextSubstitution(text=' -r -v -v1 --render-engine ogre --render-engine-gui-api-backend opengl')],
        TextSubstitution(text=' -r -v -v1')],
        'on_exit_shutdown': 'true'}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'z_pose': LaunchConfiguration('z_pose'),
            'yaw_angle': LaunchConfiguration('yaw_angle'),
        }.items()
    )

    # Launch rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_turtlebot3_mogi, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    trajectory_node = Node(
        package='mogi_trajectory_server',
        executable='mogi_trajectory_server',
        name='mogi_trajectory_server',
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(sim_time_arg)
    ld.add_action(x_pose_arg)
    ld.add_action(y_pose_arg)
    ld.add_action(z_pose_arg)
    ld.add_action(yaw_angle_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_launch_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(set_env_vars_resources)
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(rviz_node)
    ld.add_action(trajectory_node)

    return ld
