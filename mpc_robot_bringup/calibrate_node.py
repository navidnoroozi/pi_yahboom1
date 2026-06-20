#!/usr/bin/env python3
# encoding: utf-8
"""
calibrate_node.py
=================
Interactive calibration helper for the 2WD Yahboom robot. Run ONE routine at a
time, read the printed numbers, and copy the suggested values into
config/robot_params.yaml. Do this once per physical robot (motors/gearboxes
vary unit to unit).

Routines (select with the `routine` parameter):
  sign      : pulse each wheel forward for 1 s. Confirm BOTH wheels spin the
              robot FORWARD and that the printed encoder deltas are POSITIVE.
              Fix left_motor_sign / right_motor_sign / *_encoder_sign if not.
  straight  : drive both wheels at `pwm` for `seconds`. Measure the real
              distance travelled with a tape, then:
                 meters_per_tick = measured_distance_m / avg_ticks
  maxspeed  : drive both wheels at PWM 100 for `seconds`; prints measured wheel
              speed in m/s using your meters_per_tick  ->  max_wheel_speed_ms.
  spin      : spin in place; integrates heading with the current
              wheel_separation_m. Rotate exactly 360 deg (mark the floor) and
              compare printed theta to 360 deg, then correct b:
                 b_new = b_old * (printed_theta_deg / 360)

SAFETY: put the robot on blocks for `sign`/`maxspeed`, give it clear floor for
`straight`/`spin`, and keep a hand near the power switch.
"""
from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from Rosmaster_Lib import Rosmaster


def _read(car, eL, eR, esL, esR):
    m = car.get_motor_encoder()
    return esL * int(m[eL - 1]), esR * int(m[eR - 1])


class CalibrateNode(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_node")
        self.declare_parameter("serial_port", "/dev/myserial")
        self.declare_parameter("routine", "sign")     # sign|straight|maxspeed|spin
        self.declare_parameter("pwm", 30)
        self.declare_parameter("seconds", 3.0)
        # port mapping / signs (mirror driver_node params)
        self.declare_parameter("left_motor_index", 2)
        self.declare_parameter("right_motor_index", 4)
        self.declare_parameter("left_encoder_index", 2)
        self.declare_parameter("right_encoder_index", 4)
        self.declare_parameter("left_motor_sign", 1)
        self.declare_parameter("right_motor_sign", 1)
        self.declare_parameter("left_encoder_sign", 1)
        self.declare_parameter("right_encoder_sign", 1)
        # current best estimates (for maxspeed / spin)
        self.declare_parameter("meters_per_tick", 0.0001)
        self.declare_parameter("wheel_separation_m", 0.170)

        g = self.get_parameter
        self.port = str(g("serial_port").value)
        self.routine = str(g("routine").value)
        self.pwm = int(g("pwm").value)
        self.secs = float(g("seconds").value)
        self.iL, self.iR = int(g("left_motor_index").value), int(g("right_motor_index").value)
        self.eL, self.eR = int(g("left_encoder_index").value), int(g("right_encoder_index").value)
        self.sL = 1 if int(g("left_motor_sign").value) >= 0 else -1
        self.sR = 1 if int(g("right_motor_sign").value) >= 0 else -1
        self.esL = 1 if int(g("left_encoder_sign").value) >= 0 else -1
        self.esR = 1 if int(g("right_encoder_sign").value) >= 0 else -1
        self.mpt = float(g("meters_per_tick").value)
        self.b = float(g("wheel_separation_m").value)

        self.car = Rosmaster(com=self.port, debug=False)
        self.car.create_receive_threading()
        self.car.set_auto_report_state(True, forever=False)
        self.car.set_motor(0, 0, 0, 0)
        time.sleep(0.3)
        getattr(self, f"_run_{self.routine}", self._unknown)()

    def _drive(self, pwm_left, pwm_right):
        s = [0, 0, 0, 0]
        s[self.iL - 1] = int(self.sL * pwm_left)
        s[self.iR - 1] = int(self.sR * pwm_right)
        self.car.set_motor(s[0], s[1], s[2], s[3])

    def _unknown(self):
        self.get_logger().error(f"unknown routine '{self.routine}'")

    # ── sign check ───────────────────────────────────────────────────────────
    def _run_sign(self):
        for name, pl, pr in (("LEFT only", self.pwm, 0), ("RIGHT only", 0, self.pwm)):
            self.get_logger().info(f"--- {name} forward @ pwm {self.pwm} ---")
            t0L, t0R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
            self._drive(pl, pr)
            time.sleep(1.0)
            self._drive(0, 0)
            time.sleep(0.4)
            t1L, t1R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
            self.get_logger().info(
                f"    dticks  L={t1L - t0L:+d}  R={t1R - t0R:+d}  "
                f"(the driven wheel's delta should be POSITIVE; wheel should roll FORWARD)"
            )
        self.get_logger().info(
            "If a driven wheel rolled backward -> flip that *_motor_sign. "
            "If its delta was negative while rolling forward -> flip that *_encoder_sign."
        )

    # ── straight-line: meters_per_tick ───────────────────────────────────────
    def _run_straight(self):
        self.get_logger().info(
            f"Driving straight @ pwm {self.pwm} for {self.secs:.1f}s. "
            f"Measure the distance travelled with a tape."
        )
        t0L, t0R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
        self._drive(self.pwm, self.pwm)
        time.sleep(self.secs)
        self._drive(0, 0)
        time.sleep(0.5)
        t1L, t1R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
        dL, dR = t1L - t0L, t1R - t0R
        avg = 0.5 * (abs(dL) + abs(dR))
        self.get_logger().info(f"    dticks L={dL} R={dR}  avg={avg:.1f}")
        self.get_logger().info(
            f"    meters_per_tick = measured_distance_m / {avg:.1f}  "
            f"(e.g. if you measured 1.00 m -> {1.0/avg if avg else 0:.6e})"
        )
        if avg and abs(dL - dR) / avg > 0.05:
            self.get_logger().warn(
                "    L/R tick counts differ >5% -> wheels not matched; "
                "robot will veer. Consider per-wheel meters_per_tick or check wiring."
            )

    # ── max wheel speed (PWM 100) ────────────────────────────────────────────
    def _run_maxspeed(self):
        self.get_logger().info(f"Full-speed test @ pwm 100 for {self.secs:.1f}s (robot on blocks!).")
        t0L, t0R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
        self._drive(100, 100)
        time.sleep(self.secs)
        self._drive(0, 0)
        t1L, t1R = _read(self.car, self.eL, self.eR, self.esL, self.esR)
        vL = abs(t1L - t0L) * self.mpt / self.secs
        vR = abs(t1R - t0R) * self.mpt / self.secs
        self.get_logger().info(
            f"    wheel speed  L={vL:.3f} m/s  R={vR:.3f} m/s  "
            f"-> set max_wheel_speed_ms ~ {min(vL, vR):.3f}"
        )

    # ── spin: wheel_separation_m ─────────────────────────────────────────────
    def _run_spin(self):
        self.get_logger().info(
            f"Spinning in place @ pwm {self.pwm} for {self.secs:.1f}s. "
            f"Watch the robot; integrating heading with b={self.b:.3f}."
        )
        theta = 0.0
        prevL, prevR = _read(self.car, self.eL, self.eR, self.esL, self.esR)
        self._drive(-self.pwm, self.pwm)   # left back, right fwd -> CCW spin
        t_end = time.time() + self.secs
        while time.time() < t_end:
            time.sleep(0.04)
            cL, cR = _read(self.car, self.eL, self.eR, self.esL, self.esR)
            dL = (cL - prevL) * self.mpt
            dR = (cR - prevR) * self.mpt
            prevL, prevR = cL, cR
            theta += (dR - dL) / self.b
        self._drive(0, 0)
        deg = math.degrees(theta)
        self.get_logger().info(
            f"    integrated theta = {deg:.1f} deg. If the robot physically did 360 deg, "
            f"set wheel_separation_m = {self.b:.4f} * ({abs(deg):.1f}/360) "
            f"= {self.b * abs(deg) / 360.0:.4f}"
        )

    def shutdown(self):
        try:
            self.car.set_motor(0, 0, 0, 0)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateNode()
    node.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
