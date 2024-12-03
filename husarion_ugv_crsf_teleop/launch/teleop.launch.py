#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    husarion_ugv_crsf_teleop_dir = FindPackageShare("husarion_ugv_crsf_teleop")

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    husarion_ugv_crsf_teleop_node = Node(
        package="husarion_ugv_crsf_teleop",
        executable="crsf_teleop_node",
        name="crsf_teleop",
        parameters=[
            PathJoinSubstitution([husarion_ugv_crsf_teleop_dir, "config", "crsf_teleop.yaml"]),
        ],
        namespace=namespace,
        emulate_tty=True,
        respawn=True,
        respawn_delay=2,
    )

    actions = [
        declare_namespace_arg,
        husarion_ugv_crsf_teleop_node,
    ]

    return LaunchDescription(actions)
