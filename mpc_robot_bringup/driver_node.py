#!/usr/bin/env python3
# encoding: utf-8
"""
driver_node.py
==============
Per-robot hardware bring-up node for a 2WD differential-drive Yahboom robot
built on the ROS Robot Control Board V3.0 (STM32F103RCT6 + AM2861 drivers +
ICM20948 IMU), driven over USB-serial through the factory Rosmaster_Lib.

DESIGN INTENT
-------------
This node is a *drop-in hardware replacement* for ONE robot's slice of the
simulated `unicycle_fleet_node`. It honours the exact same ROS2 contract that
`formation_planner_node` already expects, so the MPC planning layer on the VM
needs **zero changes** to talk to the real robot:

    SUBSCRIBES  <ns>/cmd_vel    geometry_msgs/Twist
                                linear.x  = v   [m/s]   (forward speed)
                                angular.z = w   [rad/s] (yaw rate)

    PUBLISHES   <ns>/odom       nav_msgs/Odometry
                                pose  = (x, y, theta)  from wheel encoders
                                twist = (v_actual, w_actual)
                <ns>/imu/data_raw  sensor_msgs/Imu        (gyro + accel, no orient.)
                <ns>/imu/mag       sensor_msgs/MagneticField
                <ns>/voltage       std_msgs/Float32       (battery volts)
                <ns>/diagnostics   diagnostic_msgs/DiagnosticArray

CONTROL PATH  ***REVISED*** — firmware closed-loop, not Pi-side
-----------------------------------------------------------------
Earlier revisions of this node used `set_motor()` (raw PWM, FUNC_MOTOR) and
reimplemented a wheel-velocity PI loop on the Pi. That was based on a wrong
read of the firmware. Checking `protocol.c` directly:

    FUNC_MOTOR  (set_motor)       -> Motion_Set_Pwm()   -- open-loop PWM,
                                      the Motion_Set_Speed() call that would
                                      route it through the PID is COMMENTED
                                      OUT in firmware. "未使用编码器"
                                      (encoder not used) by design.
    FUNC_MOTION (set_car_motion)  -> Motion_Ctrl() -> {car_type}_Ctrl()
                                      -> Motion_Set_Speed() -> PID_Calc_Motor()
                                      every 10 ms in Motion_Handle(). This IS
                                      a real closed-loop wheel-velocity PID,
                                      running on the STM32, using encoder
                                      feedback (app_pid.c / app_motion.c).

So the firmware already does exactly the per-wheel velocity loop this node
used to duplicate. We now use `set_car_motion(v_x, v_y, v_z)`, which takes
m/s / rad/s directly (internally x1000, see Rosmaster_Lib.set_car_motion),
and select car_type = CAR_FOURWHEEL (0x04) at construction. Fourwheel_Ctrl()
in firmware computes a single left-side and single right-side target speed
from (Vx, Vz) -- i.e. genuine differential-drive mixing -- and applies it to
all 4 motor PID channels. With only the two wired channels (M2, M4) connected
to real motors/encoders, the other two PID channels run against an
always-zero encoder and drive nothing -- harmless.

No PI loop, no feedforward, no anti-windup logic lives on the Pi any more.
The Pi's job is: convert cmd_vel -> (v_x, v_z), send it once per control
tick, and separately read encoder ticks to integrate odometry. The firmware
wheel PID gains are tunable via `set_pid_param()` if the factory defaults
(Kp=0.8, Ki=0.06, Kd=0.5, from app_pid.h) don't suit this chassis/motor/load
-- exposed here as ROS2 parameters, pushed once at startup.

ODOMETRY (computed here — the firmware does NOT provide pose)
------------------------------------------------------------
Confirmed by inspection: car_data_t in firmware holds only {Vx, Vy, Vz}
(body velocities); Rosmaster_Lib.get_motion_data() parses the same
FUNC_REPORT_SPEED velocity triple. There is no x/y/theta integration
anywhere in firmware or the Python library. The IMU yaw (FUNC_REPORT_IMU_ATT)
is a real attitude estimate but is not a position/pose solution. So pose
still has to be integrated here, from raw encoder ticks:

    d_L = Δticks_L * meters_per_tick
    d_R = Δticks_R * meters_per_tick
    d   = (d_L + d_R)/2 ;  dθ = (d_R - d_L)/b
    x  += d*cos(θ+dθ/2) ;  y += d*sin(θ+dθ/2) ;  θ += dθ

SHARED-FRAME NOTE (important for formation control)
---------------------------------------------------
Wheel odometry starts at (0,0,0) in each robot's *own* frame and drifts
independently. The MPC assumes all robots live in ONE global frame at their
configured r0_single start poses. Set `initial_x/y/theta` per robot so each
robot's odom is expressed in the shared frame at startup. Drift is unavoidable
with wheel-only odometry — for sustained accuracy add a global reference
(overhead camera / UWB / motion capture). See ARCHITECTURE.md.

All calibration-dependent quantities (meters_per_tick, wheel_separation_m,
motor/encoder port indices and signs) are ROS2 parameters. Use
`calibrate_node` to measure them before trusting odometry. Defaults are
PLACEHOLDERS.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from tf2_ros import TransformBroadcaster

# Factory driver library (installed via: sudo python3 setup.py install)
from Rosmaster_Lib import Rosmaster

# Firmware car_type enum (app_motion.h). FOURWHEEL is the closest match to a
# 2WD-plus-caster chassis: Fourwheel_Ctrl() mixes (Vx, Vz) into one left-side
# and one right-side wheel-speed target -- true differential-drive kinematics
# -- then drives that target through the per-wheel PID on all 4 channels.
CAR_TYPE_FOURWHEEL = 0x04


def _wrap(a: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class DriverNode(Node):
    def __init__(self) -> None:
        super().__init__("driver_node")

        # ── Identity / frames ────────────────────────────────────────────────
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("publish_tf", False)   # sim publishes none; rviz=True

        # ── Serial / loop ────────────────────────────────────────────────────
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("cmd_timeout_s", 0.5)   # watchdog: stop if cmd stale

        # ── Calibrated kinematics (MEASURE THESE with calibrate_node) ─────────
        # meters_per_tick / wheel_separation_m are still needed for ODOMETRY
        # (the firmware doesn't give us pose). max_wheel_speed_ms is informational
        # only now (no longer used to compute a feedforward PWM on the Pi); kept
        # as a parameter so calibrate_node's `maxspeed` routine and any sanity
        # clamps on cmd_vel still have a reference top speed.
        self.declare_parameter("meters_per_tick", 0.0001)     # m / encoder tick
        self.declare_parameter("wheel_separation_m", 0.170)   # track width b
        self.declare_parameter("max_wheel_speed_ms", 0.70)    # informational / clamp only

        # ── Encoder port mapping: which get_motor_encoder() slot is L vs R ────
        # Yahboom advised wiring the two rear motors to M2 and M4.
        self.declare_parameter("left_encoder_index", 2)  # 1..4 (m1..m4)
        self.declare_parameter("right_encoder_index", 4) # 1..4
        self.declare_parameter("left_encoder_sign", 1)
        self.declare_parameter("right_encoder_sign", 1)

        # ── Firmware car_type + closed-loop wheel PID (runs ON the STM32) ─────
        self.declare_parameter("car_type", CAR_TYPE_FOURWHEEL)
        self.declare_parameter("push_pid_gains", False)   # set True to override defaults
        self.declare_parameter("wheel_pid_kp", 0.8)       # firmware default (app_pid.h)
        self.declare_parameter("wheel_pid_ki", 0.06)
        self.declare_parameter("wheel_pid_kd", 0.5)
        self.declare_parameter("wheel_pid_forever", False)  # True = burn to flash

        # ── cmd_vel sanity clamp (matches consensus_config u_max if desired) ──
        self.declare_parameter("v_max", 1.0)    # m/s, clamp on |linear.x|
        self.declare_parameter("w_max", 3.0)    # rad/s, clamp on |angular.z|

        # ── Shared-frame start pose (set per robot to r0_single[i]) ───────────
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_theta", 0.0)

        # ── Feature switches ─────────────────────────────────────────────────
        self.declare_parameter("publish_imu", True)
        self.declare_parameter("publish_voltage", True)
        self.declare_parameter("publish_diagnostics", True)

        g = self.get_parameter
        self.robot_id = int(g("robot_id").value)
        self.odom_frame = str(g("odom_frame").value)
        self.base_frame = str(g("base_frame").value)
        self.imu_frame = str(g("imu_frame").value)
        self.publish_tf = bool(g("publish_tf").value)

        self.serial_port = str(g("serial_port").value)
        self.rate = float(g("control_rate_hz").value)
        self.cmd_timeout = float(g("cmd_timeout_s").value)

        self.mpt = float(g("meters_per_tick").value)
        self.b = float(g("wheel_separation_m").value)
        self.vmax = float(g("max_wheel_speed_ms").value)

        self.eL = int(g("left_encoder_index").value)
        self.eR = int(g("right_encoder_index").value)
        self.esL = 1 if int(g("left_encoder_sign").value) >= 0 else -1
        self.esR = 1 if int(g("right_encoder_sign").value) >= 0 else -1

        self.car_type = int(g("car_type").value)
        self.push_pid_gains = bool(g("push_pid_gains").value)
        self.wheel_kp = float(g("wheel_pid_kp").value)
        self.wheel_ki = float(g("wheel_pid_ki").value)
        self.wheel_kd = float(g("wheel_pid_kd").value)
        self.wheel_pid_forever = bool(g("wheel_pid_forever").value)

        self.v_clamp = float(g("v_max").value)
        self.w_clamp = float(g("w_max").value)

        self.do_imu = bool(g("publish_imu").value)
        self.do_volt = bool(g("publish_voltage").value)
        self.do_diag = bool(g("publish_diagnostics").value)

        # ── Pose state (shared global frame) ─────────────────────────────────
        self.x = float(g("initial_x").value)
        self.y = float(g("initial_y").value)
        self.theta = float(g("initial_theta").value)
        self.v_actual = 0.0
        self.w_actual = 0.0

        # ── Command state ─────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.last_cmd_t = self.get_clock().now()
        self.prev_ticks_L: Optional[int] = None
        self.prev_ticks_R: Optional[int] = None
        self.prev_t = self.get_clock().now()

        # ── Connect to the board ─────────────────────────────────────────────
        self.get_logger().info(
            f"[R{self.robot_id}] opening Rosmaster on {self.serial_port} "
            f"(car_type={self.car_type}) ..."
        )
        self.car = Rosmaster(car_type=self.car_type, com=self.serial_port, debug=False)
        self.car.create_receive_threading()          # start telemetry RX thread
        self.car.set_auto_report_state(True, forever=False)
        self.car.set_car_type(self.car_type)
        self.car.set_car_motion(0.0, 0.0, 0.0)        # safe stop
        try:
            ver = self.car.get_version()
            self.get_logger().info(f"[R{self.robot_id}] board firmware v{ver}")
        except Exception:
            pass

        if self.push_pid_gains:
            self.get_logger().info(
                f"[R{self.robot_id}] pushing wheel PID gains "
                f"Kp={self.wheel_kp} Ki={self.wheel_ki} Kd={self.wheel_kd} "
                f"(forever={self.wheel_pid_forever})"
            )
            try:
                self.car.set_pid_param(
                    self.wheel_kp, self.wheel_ki, self.wheel_kd,
                    forever=self.wheel_pid_forever,
                )
            except Exception as exc:
                self.get_logger().warn(f"[R{self.robot_id}] set_pid_param failed: {exc}")

        # ── QoS: reliable depth-10 to match the sim fleet defaults ───────────
        qos = QoSProfile(depth=10,
                         reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)

        # ── Pub / Sub (relative names -> namespaced by launch) ───────────────
        self.create_subscription(Twist, "cmd_vel", self._on_cmd_vel, qos)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        if self.do_imu:
            self.imu_pub = self.create_publisher(Imu, "imu/data_raw", qos)
            self.mag_pub = self.create_publisher(MagneticField, "imu/mag", qos)
        if self.do_volt:
            self.volt_pub = self.create_publisher(Float32, "voltage", qos)
        if self.do_diag:
            self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.tf_bcast = TransformBroadcaster(self) if self.publish_tf else None

        # ── Timers ───────────────────────────────────────────────────────────
        self.create_timer(1.0 / max(self.rate, 1.0), self._control_step)
        if self.do_diag:
            self.create_timer(1.0, self._publish_diagnostics)

        self.get_logger().info(
            f"[R{self.robot_id}] driver ready | rate={self.rate:.0f} Hz | "
            f"b={self.b:.3f} m | mpt={self.mpt:.6e} m/tick | "
            f"start=({self.x:.2f},{self.y:.2f},"
            f"{math.degrees(self.theta):.0f}deg) | "
            f"control=firmware set_car_motion (CAR_TYPE={self.car_type})"
        )

    # ─────────────────────────────────────────────────────────────────────────
    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self.v_cmd = _clamp(float(msg.linear.x), -self.v_clamp, self.v_clamp)
            self.w_cmd = _clamp(float(msg.angular.z), -self.w_clamp, self.w_clamp)
            self.last_cmd_t = self.get_clock().now()

    # ─────────────────────────────────────────────────────────────────────────
    def _read_encoders(self) -> tuple[int, int]:
        """Return (left_ticks, right_ticks) with configured index + sign."""
        m = self.car.get_motor_encoder()       # (m1, m2, m3, m4) cumulative ticks
        left = self.esL * int(m[self.eL - 1])
        right = self.esR * int(m[self.eR - 1])
        return left, right

    # ─────────────────────────────────────────────────────────────────────────
    def _control_step(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.prev_t).nanoseconds * 1e-9
        self.prev_t = now
        if dt <= 1e-4 or dt > 1.0:
            # First tick or a stall: re-baseline encoders, skip this cycle.
            try:
                self.prev_ticks_L, self.prev_ticks_R = self._read_encoders()
            except Exception as exc:
                self.get_logger().warn(f"[R{self.robot_id}] encoder read failed: {exc}")
            return

        # ── 1. Read encoders -> odometry (firmware gives velocity, not pose) ──
        try:
            tL, tR = self._read_encoders()
        except Exception as exc:
            self.get_logger().warn(f"[R{self.robot_id}] encoder read failed: {exc}")
            return
        if self.prev_ticks_L is None:
            self.prev_ticks_L, self.prev_ticks_R = tL, tR
            return

        dL = (tL - self.prev_ticks_L) * self.mpt   # left wheel distance [m]
        dR = (tR - self.prev_ticks_R) * self.mpt   # right wheel distance [m]
        self.prev_ticks_L, self.prev_ticks_R = tL, tR

        d = 0.5 * (dL + dR)
        dth = (dR - dL) / self.b
        self.x += d * math.cos(self.theta + 0.5 * dth)
        self.y += d * math.sin(self.theta + 0.5 * dth)
        self.theta = _wrap(self.theta + dth)
        self.v_actual = d / dt
        self.w_actual = dth / dt

        # ── 2. Latest command + watchdog ─────────────────────────────────────
        with self._lock:
            v_cmd, w_cmd = self.v_cmd, self.w_cmd
            cmd_age = (now - self.last_cmd_t).nanoseconds * 1e-9
        if cmd_age > self.cmd_timeout:
            v_cmd = w_cmd = 0.0

        # ── 3. Hand (v, w) straight to the firmware's closed-loop controller ──
        # set_car_motion takes m/s and rad/s directly (internally x1000).
        # Fourwheel_Ctrl() does the diff-drive mixing and the per-wheel PID
        # (app_pid.c) runs on the STM32 at 10ms using encoder feedback.
        try:
            self.car.set_car_motion(v_cmd, 0.0, w_cmd)
        except Exception as exc:
            self.get_logger().warn(f"[R{self.robot_id}] set_car_motion failed: {exc}")

        # ── 4. Publish odometry (+ optional TF) ──────────────────────────────
        self._publish_odom(now)

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_odom(self, stamp_now) -> None:
        stamp = stamp_now.to_msg()
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.theta * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.theta * 0.5)
        msg.pose.covariance[0] = 0.002
        msg.pose.covariance[7] = 0.002
        msg.pose.covariance[35] = 0.01
        msg.twist.twist.linear.x = self.v_actual
        msg.twist.twist.angular.z = self.w_actual
        msg.twist.covariance[0] = 0.001
        msg.twist.covariance[35] = 0.003
        self.odom_pub.publish(msg)

        # IMU + voltage piggyback on the control timer (same rate is fine).
        if self.do_imu:
            self._publish_imu(stamp)
        if self.do_volt:
            self._publish_voltage()

        if self.tf_bcast is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = msg.pose.pose.orientation.z
            t.transform.rotation.w = msg.pose.pose.orientation.w
            self.tf_bcast.sendTransform(t)

    def _publish_imu(self, stamp) -> None:
        try:
            gx, gy, gz = self.car.get_gyroscope_data()
            ax, ay, az = self.car.get_accelerometer_data()
            mx, my, mz = self.car.get_magnetometer_data()
        except Exception:
            return
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        # NOTE: verify units/scaling against the firmware before fusing in EKF.
        imu.angular_velocity.x = float(gx)
        imu.angular_velocity.y = float(gy)
        imu.angular_velocity.z = float(gz)
        imu.linear_acceleration.x = float(ax)
        imu.linear_acceleration.y = float(ay)
        imu.linear_acceleration.z = float(az)
        imu.orientation_covariance[0] = -1.0   # no orientation provided
        self.imu_pub.publish(imu)

        mag = MagneticField()
        mag.header.stamp = stamp
        mag.header.frame_id = self.imu_frame
        mag.magnetic_field.x = float(mx)
        mag.magnetic_field.y = float(my)
        mag.magnetic_field.z = float(mz)
        self.mag_pub.publish(mag)

    def _publish_voltage(self) -> None:
        try:
            v = float(self.car.get_battery_voltage())
        except Exception:
            return
        if v > 0.0:
            self.volt_pub.publish(Float32(data=v))
            if v < 10.0:   # 3S LiPo getting low (~3.33 V/cell)
                self.get_logger().warn(
                    f"[R{self.robot_id}] LOW BATTERY {v:.2f} V — recharge soon",
                    throttle_duration_sec=30.0,
                )

    # ─────────────────────────────────────────────────────────────────────────
    def _publish_diagnostics(self) -> None:
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = f"mpc_robot/robot_{self.robot_id}"
        st.level = DiagnosticStatus.OK
        st.message = "running"
        st.values = [
            KeyValue(key="x_m", value=f"{self.x:.4f}"),
            KeyValue(key="y_m", value=f"{self.y:.4f}"),
            KeyValue(key="theta_deg", value=f"{math.degrees(self.theta):.2f}"),
            KeyValue(key="v_actual_ms", value=f"{self.v_actual:.4f}"),
            KeyValue(key="w_actual_rads", value=f"{self.w_actual:.4f}"),
        ]
        arr.status.append(st)
        self.diag_pub.publish(arr)

    # ─────────────────────────────────────────────────────────────────────────
    def shutdown(self) -> None:
        try:
            self.car.set_car_motion(0.0, 0.0, 0.0)
            self.get_logger().info(f"[R{self.robot_id}] motors stopped.")
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
