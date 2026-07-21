# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carter_nav2_bringup_dir = get_package_share_directory("carter_navigation")

    rviz_config_dir = os.path.join(
        carter_nav2_bringup_dir, "rviz2", "carter_navigation_two_robots.rviz"
    )

    robots = [{"name": "carter1"}, {"name": "carter2"}]

    ENV_MAP_FILE = "carter_warehouse_navigation.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml_file = LaunchConfiguration("map")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    autostart = LaunchConfiguration("autostart")
    log_settings = LaunchConfiguration("log_settings", default="true")

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(carter_nav2_bringup_dir, "maps", ENV_MAP_FILE),
        description="Full path to map file to load",
    )

    declare_robot1_params_file_cmd = DeclareLaunchArgument(
        "carter1_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "warehouse", "multi_robot_carter_navigation_params_1.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for carter1",
    )

    declare_robot2_params_file_cmd = DeclareLaunchArgument(
        "carter2_params_file",
        default_value=os.path.join(
            carter_nav2_bringup_dir, "params", "warehouse", "multi_robot_carter_navigation_params_2.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for carter2",
    )

    declare_bt_xml_cmd = DeclareLaunchArgument(
        "default_bt_xml_filename",
        default_value=os.path.join(
            get_package_share_directory("nav2_bt_navigator"),
            "behavior_trees",
            "navigate_w_replanning_and_recovery.xml",
        ),
        description="Full path to the behavior tree xml file to use",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart", default_value="True", description="Automatically startup the stacks"
    )

    # 두 로봇을 모두 표시하는 단일 RViz 노드
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        output="screen",
    )

    # 로봇별 네비게이션 인스턴스
    nav_instances_cmds = []
    for robot in robots:
        params_file = LaunchConfiguration(robot["name"] + "_params_file")

        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            carter_nav2_bringup_dir, "launch", "carter_navigation_individual.launch.py"
                        )
                    ),
                    launch_arguments={
                        "namespace": robot["name"],
                        "use_namespace": "True",
                        "map": map_yaml_file,
                        "use_sim_time": use_sim_time,
                        "params_file": params_file,
                        "default_bt_xml_filename": default_bt_xml_filename,
                        "autostart": autostart,
                        "use_rviz": "False",
                        "use_simulator": "False",
                        "headless": "False",
                    }.items(),
                ),
                Node(
                    package="pointcloud_to_laserscan",
                    executable="pointcloud_to_laserscan_node",
                    remappings=[
                        ("cloud_in", ["front_3d_lidar/lidar_points"]),
                        ("scan", ["scan"]),
                    ],
                    parameters=[{
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.4,
                        "max_height": 1.5,
                        "angle_min": -1.5708,
                        "angle_max": 1.5708,
                        "angle_increment": 0.0087,
                        "scan_time": 0.3333,
                        "range_min": 0.05,
                        "range_max": 100.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                    }],
                    name="pointcloud_to_laserscan",
                    namespace=robot["name"],
                ),
                LogInfo(condition=IfCondition(log_settings), msg=["Launching ", robot["name"]]),
                LogInfo(condition=IfCondition(log_settings), msg=[robot["name"], " params yaml: ", params_file]),
            ]
        )

        nav_instances_cmds.append(group)

    ld = LaunchDescription()

    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot1_params_file_cmd)
    ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(rviz_node)

    for nav_instance_cmd in nav_instances_cmds:
        ld.add_action(nav_instance_cmd)

    return ld
