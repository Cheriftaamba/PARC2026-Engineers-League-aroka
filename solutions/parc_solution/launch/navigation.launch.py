"""Bring up the navigation stack for the PARC 2026 Phase 1 task.

The competition's `task.launch.py` starts Gazebo, RViz, the robot state publisher and
the gz<->ROS bridges - no map, no localisation, no planner. Everything below is ours.

Nav2 does the planning and the driving, as the task requires. Three nodes of our own
sit around it:

  world_anchor    publishes `map -> odom` so that `map` IS the Gazebo world frame.
                  The goal is then used verbatim and never needs re-aiming, and
                  odometry drift is absorbed in the transform instead of in the goal.
  evidence_map    publishes a log-odds LiDAR occupancy grid of the restaurant, which
                  the global costmap's static layer consumes like a map_server's map:
                  obstacle memory in the world frame, plus furniture completion.
  speed_governor  publishes a nav2_msgs/SpeedLimit from the forward clearance, so the
                  speed ceiling can be raised without paying for it between the tables.

Plus scan_filter, which drops the LiDAR returns that hit the robot's own chassis.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_solution = get_package_share_directory("parc_solution")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav2_params_file = LaunchConfiguration("nav2_params_file")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use the Gazebo /clock instead of wall time.",
    )

    declare_nav2_params = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(pkg_solution, "config", "nav2_params.yaml"),
        description="Nav2 parameter file.",
    )

    # The competition's gz_bridge.yaml declares the gz topic "sitoe_robot/pose", but the
    # PosePublisher plugin publishes the model pose on "/model/sitoe_robot/pose".
    # Verified with `gz topic -e`: the declared one never delivers a single message, the
    # real one streams at 10 Hz, so the ROS topic /sitoe_robot/pose exists and is
    # permanently silent. We bridge the real one rather than patch a package the
    # organisers provided. (Worth reporting to them.)
    #
    # PoseStamped, not Pose: world_anchor pairs each pose with the odometry sample taken
    # at the same instant, and it needs the timestamp to do that.
    start_world_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="world_pose_bridge",
        output="screen",
        arguments=["/model/sitoe_robot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Localisation: `map` becomes the Gazebo world frame. See world_anchor.py.
    start_world_anchor = Node(
        package="parc_solution",
        executable="world_anchor.py",
        name="world_anchor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Must come up before Nav2: the costmaps and the collision monitor consume
    # /scan_filtered, not /scan.
    start_scan_filter = Node(
        package="parc_solution",
        executable="scan_filter.py",
        name="scan_filter",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Our perception layer, delivered to Nav2 through the static layer's map topic.
    # It publishes an (empty) map immediately, because the global costmap sizes itself
    # from this message and plans nothing at all until it arrives.
    start_evidence_map = Node(
        package="parc_solution",
        executable="evidence_map.py",
        name="evidence_map",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    start_speed_governor = Node(
        package="parc_solution",
        executable="speed_governor.py",
        name="speed_governor",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # We ship our own copy of nav2_bringup's navigation_launch.py (nav2_navigation.launch.py)
    # for two reasons that cannot be expressed through parameters: the managed-node list
    # is hard-coded there, and the cmd_vel remap has to target the collision monitor
    # specifically rather than every node in the stack.
    start_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_solution, "launch", "nav2_navigation.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
            "autostart": "true",
            "use_composition": "False",
        }.items(),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_nav2_params)
    ld.add_action(start_world_pose_bridge)
    ld.add_action(start_world_anchor)
    ld.add_action(start_scan_filter)
    ld.add_action(start_evidence_map)
    ld.add_action(start_speed_governor)
    ld.add_action(start_navigation)
    return ld
