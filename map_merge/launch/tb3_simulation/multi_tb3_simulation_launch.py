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

"""
Example for spawing multiple robots in Gazebo (gz sim).

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory("nav2_bringup")
    launch_dir = os.path.join(bringup_dir, "launch")
    sim_dir = get_package_share_directory("nav2_minimal_tb3_sim")
    # turtlebot3_house model used by the default world
    tb3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    # Get the launch directory for multirobot_map_merge where we have a modified launch files
    map_merge_dir = get_package_share_directory("multirobot_map_merge")
    launch_dir_map_merge = os.path.join(map_merge_dir, "launch", "tb3_simulation")

    # Names and poses of the robots for known poses demo
    robots_known_poses = [
        {
            "name": "robot1",
            "x_pose": 0.0,
            "y_pose": 0.5,
            "z_pose": 0.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        {
            "name": "robot2",
            "x_pose": -3.0,
            "y_pose": 1.5,
            "z_pose": 0.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    ]
    # Names and poses of the robots for unknown poses demo, the must be very close at beginning
    robots_unknown_poses = [
        {
            "name": "robot1",
            "x_pose": -2.0,
            "y_pose": 0.5,
            "z_pose": 0.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
        {
            "name": "robot2",
            "x_pose": -3.0,
            "y_pose": 0.5,
            "z_pose": 0.01,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        },
    ]

    # Simulation settings
    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")

    autostart = LaunchConfiguration("autostart")
    rviz_config_file = LaunchConfiguration("rviz_config")
    use_robot_state_pub = LaunchConfiguration("use_robot_state_pub")
    use_rviz = LaunchConfiguration("use_rviz")
    log_settings = LaunchConfiguration("log_settings", default="true")

    known_init_poses = LaunchConfiguration("known_init_poses")
    declare_known_init_poses_cmd = DeclareLaunchArgument(
        "known_init_poses",
        default_value="True",
        description="Known initial poses of the robots. If so don't forget to declare them in the params.yaml file",
    )

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(launch_dir_map_merge, "worlds", "tb3_house.sdf.xacro"),
        description="Full path to world file to load",
    )

    declare_headless_cmd = DeclareLaunchArgument(
        "headless", default_value="False", description="Whether to skip the gz sim GUI"
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        "map",
        default_value=os.path.join(bringup_dir, "maps", "tb3_sandbox.yaml"),
        description="Full path to map file to load (only used without SLAM)",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            launch_dir_map_merge, "config", "nav2_multirobot_params.yaml"
        ),
        description="Full path to the ROS2 parameters file to use for all robots",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the stacks",
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(bringup_dir, "rviz", "nav2_namespaced_view.rviz"),
        description="Full path to the RVIZ config file to use.",
    )

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        "use_robot_state_pub",
        default_value="True",
        description="Whether to start the robot state publisher",
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz", default_value="True", description="Whether to start RVIZ"
    )

    slam_toolbox = LaunchConfiguration("slam_toolbox")
    slam_gmapping = LaunchConfiguration("slam_gmapping")
    declare_slam_toolbox_cmd = DeclareLaunchArgument(
        "slam_toolbox", default_value="False", description="Whether run a SLAM toolbox"
    )
    declare_slam_gmapping_cmd = DeclareLaunchArgument(
        "slam_gmapping",
        default_value="False",
        description="Whether run a SLAM gmapping",
    )

    # gz sim only takes worlds as files, so the xacro output goes to a temporary file
    world_sdf = tempfile.mktemp(prefix="map_merge_", suffix=".sdf")
    world_sdf_xacro = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world]
    )
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf],
        output="screen",
    )
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        condition=UnlessCondition(headless),
        launch_arguments={"gz_args": ["-v4 -g "]}.items(),
    )
    # The one /clock bridge; the robots' bridges leave it out
    # (tb3_simulation/config/turtlebot3_waffle_bridge.yaml).
    clock_bridge_cmd = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    remove_temp_sdf_file = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[OpaqueFunction(function=lambda _: os.remove(world_sdf))]
        )
    )

    set_env_vars_resources = [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(sim_dir, "models")),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", str(Path(sim_dir).parent.resolve())
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(tb3_gazebo_dir, "models")
        ),
    ]

    # Spawning is done only inside tb3_simulation_launch.py, with poses passed below.

    # Define commands for launching the navigation instances
    nav_instances_cmds = []

    def _tb3_launch_arguments(robot):
        return {
            "namespace": robot["name"],
            "use_namespace": "True",
            "map": map_yaml_file,
            "use_sim_time": "True",
            "params_file": params_file,
            "autostart": autostart,
            "use_rviz": "False",
            "slam": "True",
            "slam_toolbox": slam_toolbox,
            "slam_gmapping": slam_gmapping,
            "use_robot_state_pub": use_robot_state_pub,
            "x_pose": TextSubstitution(text=str(robot["x_pose"])),
            "y_pose": TextSubstitution(text=str(robot["y_pose"])),
            "z_pose": TextSubstitution(text=str(robot["z_pose"])),
            "roll": TextSubstitution(text=str(robot["roll"])),
            "pitch": TextSubstitution(text=str(robot["pitch"])),
            "yaw": TextSubstitution(text=str(robot["yaw"])),
            "robot_name": TextSubstitution(text=robot["name"]),
        }.items()

    for robot_known, robot_unknown in zip(robots_known_poses, robots_unknown_poses):
        group = GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, "rviz_launch.py")
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        "namespace": TextSubstitution(text=robot_known["name"]),
                        "use_namespace": "True",
                        "use_sim_time": "True",
                        "rviz_config": rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir_map_merge, "tb3_simulation_launch.py")
                    ),
                    condition=IfCondition(known_init_poses),
                    launch_arguments=_tb3_launch_arguments(robot_known),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir_map_merge, "tb3_simulation_launch.py")
                    ),
                    condition=UnlessCondition(known_init_poses),
                    launch_arguments=_tb3_launch_arguments(robot_unknown),
                ),
                LogInfo(
                    condition=IfCondition(log_settings),
                    msg=["Launching ", robot_known["name"]],
                ),
            ]
        )

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    for set_env_var in set_env_vars_resources:
        ld.add_action(set_env_var)

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)
    ld.add_action(declare_slam_toolbox_cmd)
    ld.add_action(declare_slam_gmapping_cmd)
    ld.add_action(declare_known_init_poses_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(world_sdf_xacro)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(clock_bridge_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(remove_temp_sdf_file)

    ld.add_action(LogInfo(condition=IfCondition(log_settings), msg=["params yaml: ", params_file]))
    ld.add_action(LogInfo(condition=IfCondition(log_settings), msg=["autostart: ", autostart]))

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
