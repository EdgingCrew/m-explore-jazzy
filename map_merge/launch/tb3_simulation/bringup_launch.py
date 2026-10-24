# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushROSNamespace


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")

    # Get the launch directory of map_merge
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    map_merge_launch_dir = os.path.join(map_merge_dir, "launch", "tb3_simulation")

    # slam_gmapping is optional: it has no Jazzy binaries and has to be built from source
    try:
        slam_gmapping_launch_dir = os.path.join(
            get_package_share_directory("slam_gmapping"), "launch"
        )
    except PackageNotFoundError:
        slam_gmapping_launch_dir = None

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    slam = LaunchConfiguration("slam")
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run a SLAM"
    )
    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox", default_value="False", description="Whether run a SLAM toolbox"
    )
    declare_slam_gmapping_cmd = DeclareLaunchArgument(
        "slam_gmapping",
        default_value="False",
        description="Whether run a SLAM gmapping",
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map", default_value="", description="Full path to map yaml file to load (slam:=False)"
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(bringup_dir, "params", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    run_slam_toolbox = PythonExpression(
        [slam, " and ", slam_toolbox, " and not ", slam_gmapping]
    )
    run_slam_gmapping = PythonExpression(
        [slam, " and ", slam_gmapping, " and not ", slam_toolbox]
    )

    # Specify the actions
    slam_toolbox_cmd_group = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(map_merge_launch_dir, "slam_toolbox.py")
                ),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
            ),
        ],
        condition=IfCondition(run_slam_toolbox),
    )

    # Navigation only: SLAM runs above (or gmapping below), so nav2's own
    # slam_launch.py / AMCL stay off. nav2_bringup pushes the namespace itself.
    # Scoped group: the include writes its arguments into the launch context,
    # and "slam" must not leak into the gmapping condition below.
    navigation_cmd = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, "bringup_launch.py")
                ),
                # Arguments are evaluated in order: use_localization must read our
                # "slam" before it is overwritten below.
                launch_arguments=[
                    ("use_localization", PythonExpression(["not ", slam])),
                    ("slam", "False"),
                    ("namespace", namespace),
                    ("use_namespace", use_namespace),
                    ("map", map_yaml_file),
                    ("use_sim_time", use_sim_time),
                    ("autostart", autostart),
                    ("params_file", params_file),
                ],
            ),
        ]
    )

    if slam_gmapping_launch_dir is not None:
        # Not in GroupAction because namespace were prepended twice because
        # slam_gmapping.launch.py already accepts a namespace argument
        slam_gmapping_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_gmapping_launch_dir, "slam_gmapping.launch.py")
            ),
            condition=IfCondition(run_slam_gmapping),
            launch_arguments={
                "namespace": namespace,
                "use_sim_time": use_sim_time,
            }.items(),
        )
    else:
        slam_gmapping_cmd = LogInfo(
            condition=IfCondition(run_slam_gmapping),
            msg="slam_gmapping:=True but the slam_gmapping package was not found; no SLAM is running",
        )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_slam_gmapping_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(slam_toolbox_cmd_group)
    ld.add_action(navigation_cmd)
    ld.add_action(slam_gmapping_cmd)

    return ld
