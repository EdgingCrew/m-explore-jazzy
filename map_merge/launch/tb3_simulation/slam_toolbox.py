import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params_file = LaunchConfiguration("slam_params_file")
    autostart = LaunchConfiguration("autostart")

    remappings = [
        ("/map", "map"),
        ("/scan", "scan"),
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("slam_toolbox"),
            "config",
            "mapper_params_online_sync.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )
    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Configure and activate slam_toolbox on startup",
    )

    # Since Jazzy slam_toolbox is a lifecycle node: it stays unconfigured (no /map)
    # until it is configured and activated.
    start_sync_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {"use_lifecycle_manager": False, "use_sim_time": use_sim_time},
        ],
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        remappings=remappings,
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_sync_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_sync_slam_toolbox_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        ),
        condition=IfCondition(autostart),
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(start_sync_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)

    return ld
