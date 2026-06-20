#!/usr/bin/env python3
"""
path_follower_node.py — Waypoint path follower for robot1
----------------------------------------------------------
Runs on the RPi4. Subscribes to /odom/unfiltered, publishes /cmd_vel.

USAGE — Test 1 (run entirely on RPi4):
    python3 path_follower_node.py --mode local

    When the node is ready you will see:
        "Trigger: ros2 topic pub /path_start std_msgs/msg/Empty {} --once"
    Run that command in another terminal to start the robot.

USAGE — Test 2 (triggered from Ubuntu VM via ZMQ):
    python3 path_follower_node.py --mode zmq --zmq-port 5560

    Then on the VM:
        python3 vm_path_trigger.py --robot-ip <rpi4_ip>

Corrections applied vs original:
  1. Odom source:    /odom/unfiltered (BEST_EFFORT) instead of /odom
                     Avoids EKF gyro-bias drift corrupting position estimate.
  2. Start trigger:  ROS2 topic /path_start instead of stdin input().
                     stdin in a daemon thread consumed buffered newlines
                     and started the robot immediately without user input.
  3. Odom stability: Start is only accepted after 3 s of stable odom.
                     Prevents the path from starting before the EKF /
                     serial bridge has initialised, which caused the robot
                     to never reach waypoints (position stuck at 0,0).
  4. QoS:           Removed incorrect BEST_EFFORT QoS on the subscriber.
                     /odom/unfiltered is published BEST_EFFORT so the
                     subscriber uses the default (depth=10) which is
                     compatible.

The path is a rectangle that fits in a 3 m × 2 m lab space:
    (0,0) → (1.0,0) → (1.0,0.6) → (0,0.6) → (0,0)
    Total perimeter ≈ 4.6 m.

State machine per waypoint:
    WAITING → ROTATING → DRIVING → PAUSING → (next waypoint) → DONE
"""

import argparse
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


# ── Path definition ───────────────────────────────────────────────────────────
# Rectangle in the odom frame, robot starts at origin facing +X.
WAYPOINTS = [
    (1.00, 0.00),   # leg 1: drive 1.0 m forward
    (1.00, 0.60),   # leg 2: turn left, drive 0.6 m
    (0.00, 0.60),   # leg 3: turn left, drive 1.0 m back
    (0.00, 0.00),   # leg 4: turn left, return to origin
]

# ── Controller parameters ─────────────────────────────────────────────────────
WAYPOINT_TOLERANCE  = 0.08   # m   — declare waypoint reached within 8 cm
HEADING_TOLERANCE   = 0.10   # rad — ~5.7°, transition ROTATING → DRIVING
MAX_LINEAR_SPEED    = 0.12   # m/s
MAX_ANGULAR_SPEED   = 0.50   # rad/s
KP_LINEAR           = 1.00   # proportional gain: forward speed vs distance
KP_ANGULAR          = 1.80   # proportional gain: angular speed vs heading error
STOP_DURATION       = 1.0    # s   — pause at each waypoint before next leg
ODOM_STABLE_DELAY   = 3.0    # s   — wait for odom to stabilise before start

# ── State machine states ──────────────────────────────────────────────────────
WAITING  = 'WAITING'
ROTATING = 'ROTATING'
DRIVING  = 'DRIVING'
PAUSING  = 'PAUSING'
DONE     = 'DONE'


def angle_wrap(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def quaternion_to_yaw(qx, qy, qz, qw) -> float:
    """Extract yaw angle from quaternion."""
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


class PathFollowerNode(Node):

    def __init__(self, mode: str, zmq_port: int):
        super().__init__('path_follower_node')

        self.mode    = mode
        self.started = False       # set True by start trigger
        self.state   = WAITING
        self.wp_idx  = 0
        self.pause_t = None

        # Pose — updated from odom callback
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0

        # Odom stability tracking (fix 3)
        self.odom_received     = False
        self.odom_stable_since = None  # timestamp of first valid odom message

        # ── Subscriber: /odom/unfiltered (fix 1 + fix 4) ─────────────────
        # /odom/unfiltered is published BEST_EFFORT by serial_bridge_node.
        # Using pure wheel odometry avoids EKF gyro-drift corrupting position.
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom/unfiltered', self._odom_cb, odom_qos)

        # ── Publisher ─────────────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ── Start trigger: /path_start topic (fix 2) ─────────────────────
        # In local mode, publish from a second terminal:
        #   ros2 topic pub /path_start std_msgs/msg/Empty {} --once
        # This avoids the stdin buffering bug where input() consumed a
        # previously-buffered newline and started the robot immediately.
        if mode == 'local':
            self.start_sub = self.create_subscription(
                Empty, '/path_start', self._start_callback, 10)

        # ── ZMQ start trigger (Test 2 only) ───────────────────────────────
        if mode == 'zmq':
            self._start_zmq_listener(zmq_port)

        # ── Control loop at 20 Hz ─────────────────────────────────────────
        self.timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info(
            f'Path follower ready — mode={mode}, '
            f'{len(WAYPOINTS)} waypoints')

        if mode == 'local':
            self.get_logger().info(
                '\n' + '='*60 +
                '\n  PATH FOLLOWER — Test 1 (local)' +
                '\n  Path: rectangle 1.0 m × 0.6 m' +
                '\n  1. Place robot at start position facing forward (+X).' +
                f'\n  2. Wait for "odom stable" message (~{ODOM_STABLE_DELAY:.0f}s).' +
                '\n  3. In another terminal run:' +
                '\n       ros2 topic pub /path_start std_msgs/msg/Empty {} --once' +
                '\n' + '='*60)

    # ── Odom callback ─────────────────────────────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        if not self.odom_received:
            self.odom_received     = True
            self.odom_stable_since = time.time()
            self.get_logger().info(
                f'First odom received — waiting {ODOM_STABLE_DELAY:.0f}s '
                f'for stability before accepting start trigger')

    # ── Start callbacks ───────────────────────────────────────────────────────
    def _start_callback(self, _msg):
        """Called when /path_start topic is published (local mode)."""
        self._try_start()

    def _try_start(self):
        """Attempt to start — rejected if odom not yet stable."""
        if self.started:
            return

        if not self.odom_received:
            self.get_logger().warn(
                'Start trigger received but odom not yet available — '
                'is bringup running?')
            return

        elapsed = time.time() - self.odom_stable_since
        if elapsed < ODOM_STABLE_DELAY:
            remaining = ODOM_STABLE_DELAY - elapsed
            self.get_logger().warn(
                f'Start trigger received too early — '
                f'waiting {remaining:.1f}s more for odom stability')
            return

        self.started = True
        self.get_logger().info(
            f'START accepted — odom stable for {elapsed:.1f}s. '
            f'Beginning path with {len(WAYPOINTS)} waypoints.')

    # ── Main control loop ─────────────────────────────────────────────────────
    def _control_loop(self):
        # Do nothing until first odom message arrives
        if not self.odom_received:
            return

        # Print "odom stable" message once, 3 seconds after first odom
        if (self.odom_stable_since is not None and
                not self.started and
                not hasattr(self, '_stable_notified') and
                time.time() - self.odom_stable_since >= ODOM_STABLE_DELAY):
            self._stable_notified = True
            self.get_logger().info(
                '✓ Odom stable — ready to start. '
                'Publish /path_start when robot is in position.')

        # ── WAITING: do nothing until start trigger ────────────────────────
        if self.state == WAITING:
            if self.started:
                self.state = ROTATING
                wx, wy = WAYPOINTS[0]
                self.get_logger().info(
                    f'→ Waypoint 1/{len(WAYPOINTS)}: '
                    f'({wx:.2f}, {wy:.2f})')
            return

        # ── DONE: keep publishing zero to ensure robot is stopped ──────────
        if self.state == DONE:
            self._publish_stop()
            return

        # ── PAUSING: wait at waypoint before continuing ────────────────────
        if self.state == PAUSING:
            self._publish_stop()
            if time.time() - self.pause_t >= STOP_DURATION:
                self.wp_idx += 1
                if self.wp_idx >= len(WAYPOINTS):
                    self.state = DONE
                    self.get_logger().info(
                        '✓ Path complete — all waypoints reached')
                else:
                    self.state = ROTATING
                    wx, wy = WAYPOINTS[self.wp_idx]
                    self.get_logger().info(
                        f'→ Waypoint {self.wp_idx+1}/{len(WAYPOINTS)}: '
                        f'({wx:.2f}, {wy:.2f})')
            return

        # ── Compute distance and heading to current target waypoint ────────
        wx, wy     = WAYPOINTS[self.wp_idx]
        dx         = wx - self.x
        dy         = wy - self.y
        dist       = math.sqrt(dx*dx + dy*dy)

        # ── Waypoint reached check ─────────────────────────────────────────
        if dist < WAYPOINT_TOLERANCE:
            self._publish_stop()
            self.state   = PAUSING
            self.pause_t = time.time()
            self.get_logger().info(
                f'✓ Waypoint {self.wp_idx+1} reached: '
                f'x={self.x:.3f} y={self.y:.3f}  '
                f'error={dist*100:.1f} cm')
            return

        desired_heading = math.atan2(dy, dx)
        heading_error   = angle_wrap(desired_heading - self.theta)
        twist           = Twist()

        # ── ROTATING: turn in place until heading is aligned ───────────────
        if self.state == ROTATING:
            if abs(heading_error) < HEADING_TOLERANCE:
                self.state = DRIVING
                # Fall through to DRIVING immediately (no return)
            else:
                twist.angular.z = max(
                    -MAX_ANGULAR_SPEED,
                    min(MAX_ANGULAR_SPEED,
                        KP_ANGULAR * heading_error))
                self.cmd_pub.publish(twist)
                return

        # ── DRIVING: move toward waypoint with heading correction ──────────
        if self.state == DRIVING:
            # If heading drifts too far, go back to fine-tuning
            if abs(heading_error) > 3.0 * HEADING_TOLERANCE:
                self.state = ROTATING
                self._publish_stop()
                return

            # Proportional speed: slow down as we approach
            speed = min(MAX_LINEAR_SPEED, KP_LINEAR * dist)
            twist.linear.x  = speed
            twist.angular.z = max(
                -MAX_ANGULAR_SPEED,
                min(MAX_ANGULAR_SPEED,
                    KP_ANGULAR * heading_error))
            self.cmd_pub.publish(twist)

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())

    # ── ZMQ mode: listen for start command from VM ────────────────────────────
    def _start_zmq_listener(self, port: int):
        try:
            import zmq
        except ImportError:
            self.get_logger().error(
                'zmq not installed: '
                'pip3 install pyzmq --break-system-packages')
            return

        def _listen():
            ctx  = zmq.Context.instance()
            sock = ctx.socket(zmq.REP)
            sock.bind(f'tcp://0.0.0.0:{port}')
            self.get_logger().info(
                f'ZMQ listener on port {port} — waiting for START from VM')

            while rclpy.ok():
                try:
                    msg = sock.recv_string(flags=zmq.NOBLOCK)
                except zmq.Again:
                    time.sleep(0.05)
                    continue

                if msg == 'START':
                    self._try_start()
                    sock.send_string('OK' if self.started else 'NOT_READY')
                elif msg == 'STATUS':
                    elapsed = (time.time() - self.odom_stable_since
                               if self.odom_stable_since else 0.0)
                    sock.send_string(
                        f'{self.state}:{self.wp_idx}:'
                        f'odom_age={elapsed:.1f}s')
                elif msg == 'STOP':
                    self.state = DONE
                    self._publish_stop()
                    sock.send_string('OK')
                else:
                    sock.send_string('UNKNOWN')

        t = threading.Thread(target=_listen, daemon=True)
        t.start()

    def destroy_node(self):
        """Publish zero velocity before shutting down."""
        try:
            self._publish_stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='Waypoint path follower for robot1')
    parser.add_argument(
        '--mode', choices=['local', 'zmq'], default='local',
        help='local = triggered by /path_start topic; '
             'zmq = triggered from Ubuntu VM')
    parser.add_argument(
        '--zmq-port', type=int, default=5560,
        help='ZMQ REP port for VM trigger (zmq mode only, default 5560)')
    args = parser.parse_args()

    rclpy.init()
    node = PathFollowerNode(mode=args.mode, zmq_port=args.zmq_port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._publish_stop()
        node.get_logger().info('Stopped by user (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()