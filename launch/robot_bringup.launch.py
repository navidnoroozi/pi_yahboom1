#!/usr/bin/env python3
"""
robot_bringup.launch.py
=======================
Bring up ONE physical robot. Run this on each robot's Raspberry Pi 4.

Usage (on robot i's RPi4):
    ros2 launch mpc_robot_bringup robot_bringup.launch.py robot_id:=1
    ros2 launch mpc_robot_bringup robot_bringup.launch.py robot_id:=2 serial_port:=/dev/myserial

What it does
------------
  * pushes namespace  robot_<id>   -> topics become /robot_<id>/cmd_vel, /odom, ...
  * loads config/robot_params.yaml  (calibration: meters_per_tick, b, vmax, signs)
  * sets the shared-frame start pose for this robot from r0_single (so all
    robots' odometry lives in the SAME global frame the MPC assumes).

Override the start pose if your physical layout differs:
    ... robot_id:=1 initial_x:=-2.0 initial_y:=7.0 initial_theta:=0.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


# Default shared-frame start poses == consensus_config.r0_single (single_integrator).
# Keep these in sync with consensus_config.py on the VM.
R0_SINGLE = {
    1: (-2.0, 7.0, 0.0),
    2: (7.5, 4.0, 0.0),
    3: (4.5, -4.5, 0.0),
    4: (-3.5, -4.0, 0.0),
}


def _setup(context, *args, **kwargs):
    rid = int(LaunchConfiguration("robot_id").perform(context))
    serial_port = LaunchConfiguration("serial_port").perform(context)
    params_file = LaunchConfiguration("params_file").perform(context)

    # Resolve initial pose: explicit launch arg wins, else r0_single default.
    def _pose(arg_name, default):
        val = LaunchConfiguration(arg_name).perform(context)
        return float(val) if val != "" else float(default)

    dx, dy, dth = R0_SINGLE.get(rid, (0.0, 0.0, 0.0))
    x0 = _pose("initial_x", dx)
    y0 = _pose("initial_y", dy)
    th0 = _pose("initial_theta", dth)

    driver = Node(
        package="mpc_robot_bringup",
        executable="driver_node",
        name="driver_node",
        output="screen",
        parameters=[
            params_file,
            {
                "robot_id": rid,
                "serial_port": serial_port,
                "initial_x": x0,
                "initial_y": y0,
                "initial_theta": th0,
            },
        ],
    )

    return [GroupAction([PushRosNamespace(f"robot_{rid}"), driver])]


def generate_launch_description():
    pkg = get_package_share_directory("mpc_robot_bringup")
    default_params = os.path.join(pkg, "config", "robot_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="1",
                              description="Robot index 1..n (sets namespace + start pose)"),
        DeclareLaunchArgument("serial_port", default_value="/dev/myserial",
                              description="Serial device for the ROS Control Board V3.0"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="Calibration parameter YAML for this robot"),
        DeclareLaunchArgument("initial_x", default_value="",
                              description="Override shared-frame start x (m)"),
        DeclareLaunchArgument("initial_y", default_value="",
                              description="Override shared-frame start y (m)"),
        DeclareLaunchArgument("initial_theta", default_value="",
                              description="Override shared-frame start heading (rad)"),
        OpaqueFunction(function=_setup),
    ])
