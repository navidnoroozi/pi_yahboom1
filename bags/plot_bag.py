# plot_bag.py — plot multiple ROS 2 Jazzy MCAP topics from a bag

import math
import pathlib
from collections import Counter

import matplotlib
matplotlib.use("Agg")  # no display needed
import matplotlib.pyplot as plt

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


# ============================================================
# Configuration
# ============================================================
bag_path = pathlib.Path("/home/pi/robot_ws/src/robot_bringup/bags/robot_run")
output_dir = pathlib.Path.home() / "robot_ws/src/robot_bringup/artifacts/bag_plots"
output_dir.mkdir(exist_ok=True)

selected_topics = [
    "/cmd_vel",
    "/odom/unfiltered",
    "/imu/data",
    "/diagnostics",
    "/odom",
    "/rosout",
]


# ============================================================
# Helper functions
# ============================================================
def byte_or_int_to_int(value):
    """Convert ROS byte/uint8 fields to normal Python int."""
    if isinstance(value, (bytes, bytearray)):
        return value[0]
    return int(value)


def quat_to_yaw(x, y, z, w):
    """Convert quaternion to yaw angle [rad]."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def save_plot(filename, title, xlabel, ylabel):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    out = output_dir / filename
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out}")


def save_plot_no_legend(filename, title, xlabel, ylabel):
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.tight_layout()
    out = output_dir / filename
    plt.savefig(out, dpi=150, bbox_inches="tight")
    plt.close()
    print(f"Saved: {out}")


def level_to_name_rosout(level):
    return {
        10: "DEBUG",
        20: "INFO",
        30: "WARN",
        40: "ERROR",
        50: "FATAL",
    }.get(level, f"UNKNOWN({level})")


def level_to_name_diag(level):
    return {
        0: "OK",
        1: "WARN",
        2: "ERROR",
        3: "STALE",
    }.get(level, f"UNKNOWN({level})")


# ============================================================
# Prepare bag reader
# ============================================================
if not bag_path.exists():
    raise FileNotFoundError(f"Bag folder does not exist: {bag_path}")

reader = rosbag2_py.SequentialReader()

storage_options = rosbag2_py.StorageOptions(
    uri=str(bag_path),
    storage_id="mcap",
)

converter_options = rosbag2_py.ConverterOptions(
    input_serialization_format="cdr",
    output_serialization_format="cdr",
)

reader.open(storage_options, converter_options)

topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

print("Available topics in bag:")
for name, typ in type_map.items():
    print(f"  {name}: {typ}")

msg_type_map = {}
for topic_name in selected_topics:
    if topic_name in type_map:
        msg_type_map[topic_name] = get_message(type_map[topic_name])

# ============================================================
# Data containers
# ============================================================
t0 = None

# /cmd_vel
cmd_t = []
cmd_lin_x = []
cmd_ang_z = []

# /odom
odom_t = []
odom_x = []
odom_y = []
odom_yaw = []
odom_speed = []

# /odom/unfiltered
odom_u_t = []
odom_u_x = []
odom_u_y = []
odom_u_yaw = []
odom_u_speed = []

# /imu/data
imu_t = []
imu_ax = []
imu_ay = []
imu_az = []
imu_gx = []
imu_gy = []
imu_gz = []

# /diagnostics
diag_t = []
diag_level = []
diag_name = []

# /rosout
rosout_t = []
rosout_level = []
rosout_name = []
rosout_msg = []


# ============================================================
# Read the bag once
# ============================================================
while reader.has_next():
    topic, data, timestamp = reader.read_next()

    if topic not in msg_type_map:
        continue

    if t0 is None:
        t0 = timestamp

    t = (timestamp - t0) * 1e-9  # ns -> s
    msg = deserialize_message(data, msg_type_map[topic])

    # ---------------- /cmd_vel ----------------
    if topic == "/cmd_vel":
        cmd_t.append(t)
        cmd_lin_x.append(msg.linear.x)
        cmd_ang_z.append(msg.angular.z)

    # ---------------- /odom ----------------
    elif topic == "/odom":
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        odom_t.append(t)
        odom_x.append(x)
        odom_y.append(y)
        odom_yaw.append(yaw)
        odom_speed.append(speed)

    # ---------------- /odom/unfiltered ----------------
    elif topic == "/odom/unfiltered":
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        speed = math.sqrt(vx * vx + vy * vy)

        odom_u_t.append(t)
        odom_u_x.append(x)
        odom_u_y.append(y)
        odom_u_yaw.append(yaw)
        odom_u_speed.append(speed)

    # ---------------- /imu/data ----------------
    elif topic == "/imu/data":
        imu_t.append(t)
        imu_ax.append(msg.linear_acceleration.x)
        imu_ay.append(msg.linear_acceleration.y)
        imu_az.append(msg.linear_acceleration.z)

        imu_gx.append(msg.angular_velocity.x)
        imu_gy.append(msg.angular_velocity.y)
        imu_gz.append(msg.angular_velocity.z)

    # ---------------- /diagnostics ----------------
    elif topic == "/diagnostics":
        for status in msg.status:
            diag_t.append(t)
            diag_level.append(byte_or_int_to_int(status.level))
            diag_name.append(status.name)

    # ---------------- /rosout ----------------
    elif topic == "/rosout":
        rosout_t.append(t)
        rosout_level.append(byte_or_int_to_int(msg.level))
        rosout_name.append(msg.name)
        rosout_msg.append(msg.msg)


# ============================================================
# Create plots
# ============================================================

# ---------------- /cmd_vel ----------------
if cmd_t:
    plt.figure()
    plt.plot(cmd_t, cmd_lin_x, label="linear.x [m/s]")
    plt.plot(cmd_t, cmd_ang_z, label="angular.z [rad/s]")
    save_plot(
        "cmd_vel_timeseries.png",
        "/cmd_vel over time",
        "time [s]",
        "value"
    )

# ---------------- /odom ----------------
if odom_t:
    plt.figure()
    plt.plot(odom_x, odom_y, label="/odom trajectory")
    plt.axis("equal")
    save_plot(
        "odom_xy.png",
        "/odom trajectory",
        "x position [m]",
        "y position [m]"
    )

    plt.figure()
    plt.plot(odom_t, odom_x, label="x [m]")
    plt.plot(odom_t, odom_y, label="y [m]")
    plt.plot(odom_t, odom_yaw, label="yaw [rad]")
    plt.plot(odom_t, odom_speed, label="speed [m/s]")
    save_plot(
        "odom_timeseries.png",
        "/odom states over time",
        "time [s]",
        "value"
    )

# ---------------- /odom/unfiltered ----------------
if odom_u_t:
    plt.figure()
    plt.plot(odom_u_x, odom_u_y, label="/odom/unfiltered trajectory")
    plt.axis("equal")
    save_plot(
        "odom_unfiltered_xy.png",
        "/odom/unfiltered trajectory",
        "x position [m]",
        "y position [m]"
    )

    plt.figure()
    plt.plot(odom_u_t, odom_u_x, label="x [m]")
    plt.plot(odom_u_t, odom_u_y, label="y [m]")
    plt.plot(odom_u_t, odom_u_yaw, label="yaw [rad]")
    plt.plot(odom_u_t, odom_u_speed, label="speed [m/s]")
    save_plot(
        "odom_unfiltered_timeseries.png",
        "/odom/unfiltered states over time",
        "time [s]",
        "value"
    )

# ---------------- /imu/data ----------------
if imu_t:
    plt.figure()
    plt.plot(imu_t, imu_ax, label="accel.x [m/s^2]")
    plt.plot(imu_t, imu_ay, label="accel.y [m/s^2]")
    plt.plot(imu_t, imu_az, label="accel.z [m/s^2]")
    save_plot(
        "imu_linear_acceleration.png",
        "/imu/data linear acceleration",
        "time [s]",
        "acceleration [m/s^2]"
    )

    plt.figure()
    plt.plot(imu_t, imu_gx, label="gyro.x [rad/s]")
    plt.plot(imu_t, imu_gy, label="gyro.y [rad/s]")
    plt.plot(imu_t, imu_gz, label="gyro.z [rad/s]")
    save_plot(
        "imu_angular_velocity.png",
        "/imu/data angular velocity",
        "time [s]",
        "angular velocity [rad/s]"
    )

# ---------------- /diagnostics ----------------
if diag_t:
    plt.figure()
    plt.plot(diag_t, diag_level, "o", label="diagnostic level")
    plt.yticks([0, 1, 2, 3], ["OK", "WARN", "ERROR", "STALE"])
    save_plot(
        "diagnostics_levels_over_time.png",
        "/diagnostics levels over time",
        "time [s]",
        "diagnostic level"
    )

    diag_counts = Counter(diag_name)
    plt.figure()
    plt.bar(list(diag_counts.keys()), list(diag_counts.values()), label="count")
    plt.xticks(rotation=45, ha="right")
    save_plot(
        "diagnostics_status_counts.png",
        "/diagnostics status counts",
        "status name",
        "count"
    )

# ---------------- /rosout ----------------
if rosout_t:
    plt.figure()
    plt.plot(rosout_t, rosout_level, "o", label="rosout level")
    plt.yticks(
        [10, 20, 30, 40, 50],
        ["DEBUG", "INFO", "WARN", "ERROR", "FATAL"]
    )
    save_plot(
        "rosout_levels_over_time.png",
        "/rosout levels over time",
        "time [s]",
        "log level"
    )

    rosout_counts = Counter(level_to_name_rosout(lvl) for lvl in rosout_level)
    plt.figure()
    plt.bar(list(rosout_counts.keys()), list(rosout_counts.values()), label="count")
    save_plot(
        "rosout_level_counts.png",
        "/rosout level counts",
        "log level",
        "count"
    )

    rosout_txt = output_dir / "rosout_messages.txt"
    with rosout_txt.open("w", encoding="utf-8") as f:
        for t, lvl, name, msg in zip(rosout_t, rosout_level, rosout_name, rosout_msg):
            f.write(f"{t:10.3f} s  [{level_to_name_rosout(lvl)}]  {name}: {msg}\n")
    print(f"Saved: {rosout_txt}")

print("\nDone. All plots were written to:")
print(output_dir)
