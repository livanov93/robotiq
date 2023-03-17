# Copyright 2023 PickNik Inc
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
# Author: Lovro Ivanov
#

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

from launch.actions import OpaqueFunction
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):

    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    sim_ignition = LaunchConfiguration("sim_ignition", default=True)
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    start_rviz = LaunchConfiguration("start_rviz")
    gripper_controller = LaunchConfiguration("gripper_controller")

    # Get URDF via xacro - doesn't work because of the mesh paths
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("robotiq_2f_85_gripper_visualization"),
    #              "urdf",
    #              "robotiq_gripper.urdf.xacro"]
    #         ),
    #         " ",
    #         "prefix:=",
    #         prefix,
    #         " ",
    #         "use_fake_hardware:=",
    #         use_fake_hardware,
    #         " ",
    #         "fake_sensor_commands:=",
    #         fake_sensor_commands,
    #         " ",
    #         "sim_ignition:=",
    #         sim_ignition,
    #     ]
    # )

    description_package_path = os.path.join(
        get_package_share_directory('robotiq_2f_85_gripper_visualization'))

    urdf_file = os.path.join(description_package_path,
                              'urdf',
                              'robotiq_gripper.urdf')

    doc = xacro.parse(open(urdf_file))
    xacro.process_doc(doc)

    robot_description_content_ignition = doc.toxml()

    robot_description_params = {'robot_description': robot_description_content_ignition}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_params, {"use_sim_time": True}]
    )

    ignition_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_description_content_ignition,
                   '-name', 'robotiq_gripper',
                   '-allow_renaming', 'true',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '1.3',
                   '-R', '0.0',
                   '-P', '3.14',
                   '-Y', '0.0'
                   ],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             gripper_controller.perform(context)],
        output='screen'
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("robotiq_2f_85_gripper_visualization"), "config", "visualize.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    nodes_to_start = [
        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ros_gz_sim'),
                              'launch', 'gz_sim.launch.py')]),
            launch_arguments=[('gz_args', [' -r -v 4 pick_cube.sdf'])]),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=ignition_spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_gripper_controller],
            )
        ),
        node_robot_state_publisher,
        ignition_spawn_entity,
        rviz_node,
        clock_bridge,
        # Launch Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'),
    ]

    return nodes_to_start


def generate_launch_description():
    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim_ignition",
            default_value="true",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gripper_controller",
            default_value="robotiq_gripper_controller",
            description="Robot controller to start.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
