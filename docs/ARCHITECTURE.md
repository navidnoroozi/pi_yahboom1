# Architecture — Distributed MPC on 4 Yahboom 2WD Robots

This document describes the hardware/software stack that connects your existing
distributed hybrid-MPC formation controller (running on the Ubuntu VM) to four
physical 2WD Yahboom differential-drive robots, each carrying a Raspberry Pi 4
and a Yahboom **ROS Robot Control Board V3.0** (STM32F103RCT6 + AM2861 motor
drivers + ICM20948 IMU) running the **factory pre-flashed firmware**.

---

## 1. The decision: build fresh, don't adapt `yahboomcar_bringup`

You asked whether to (a) adapt Yahboom's `yahboomcar_bringup` or (b) write a
fresh package on top of `Rosmaster_Lib`. **Recommendation: write fresh (option b),
using `yahboomcar_bringup` only as a pattern reference.** Reasons:

1. **`yahboomcar_bringup` is the wrong shape.** Its drivers are car-type
   specific — `Mcnamu_driver_X3` (Mecanum), `Ackman_driver_R2` (Ackermann with a
   steering servo), `Mcnamu_driver_x1` (4WD). They publish 6-DOF arm/steer
   `JointState`, interpret `linear.y` as a steering angle, and assume Yahboom's
   exact chassis footprint and URDF. For a 2WD diff robot you would delete more
   than you keep, and the leftovers (steer joints, `vy*1000` steering hacks)
   invite subtle bugs.

2. **Your robot interface is trivially simple** — a unicycle. It is much easier
   to verify a 120-line purpose-built node than to audit which parts of the
   Yahboom drivers still apply. That matches how you've built every other layer
   of this project (build-and-verify, minimal moving parts).

3. **You should own the wheel loop.** As a control engineer you want explicit
   authority over the inverse kinematics and the wheel-velocity loop, not a
   black box buried in firmware kinematics keyed to a `car_type` that doesn't
   match your chassis.

4. **The interface contract is already fixed by your simulator** (Section 3).
   The cleanest possible deliverable is a node that satisfies that exact
   contract, so the MPC layer needs zero changes.

What we **keep** from Yahboom as reference: the pattern of wrapping `Rosmaster`
in one node; publishing IMU/voltage; the EKF + `imu_filter_madgwick` pipeline
(optional, Section 7); and the calibration methodology from
`calibrate_linear_*` / `calibrate_angular_*` (folded into `calibrate_node`).

---

## 2. A correction worth making explicit (the firmware does NOT give odometry)

It's worth being precise about what the pre-flashed firmware actually provides,
because it shapes the whole design:

* The board speaks a **custom serial protocol over USB**, *not* ROS2. ROS2
  topics only exist on the Pi, created by your bring-up node. `Rosmaster_Lib`
  wraps that serial protocol.
* What the firmware reports (auto-reported ~every 40 ms): **per-wheel encoder
  counts**, a **body-velocity estimate** (`get_motion_data`, computed with the
  firmware's `car_type` kinematics), and **raw + attitude-fused IMU** data
  (ICM20948 → roll/pitch/yaw).
* What it does **not** provide: **pose odometry `(x, y, θ)`**. There is no
  odometry method in `Rosmaster_Lib`. In Yahboom's own stack the *ROS layer*
  (`base_node_*`) integrates velocity into odometry, optionally fused with the
  IMU by `robot_localization`.

So "the firmware provides state estimation directly" is only half true: it gives
you filtered IMU + wheel feedback, but **pose odometry must be computed on the
Pi**. `driver_node` computes it from raw encoder ticks using *your* calibrated
geometry — fully transparent, and independent of the firmware's car-type model.

---

## 3. The interface contract (fixed by `unicycle_fleet_node`)

Your `formation_planner_node` already does the single-integrator → unicycle
conversion (Cartesian `u_safe` → `(v, ω)` via its heading controller) and talks
to the robots over plain ROS2 topics. The simulated `unicycle_fleet_node`
defines the contract each robot must satisfy:

```
SUBSCRIBE  /robot_i/cmd_vel   geometry_msgs/Twist
                              linear.x  = v  [m/s]
                              angular.z = ω  [rad/s]

PUBLISH    /robot_i/odom      nav_msgs/Odometry
                              pose  = x, y, θ(quaternion z,w)
                              twist = v_actual, ω_actual
           /robot_i/diagnostics  diagnostic_msgs/DiagnosticArray
```

**`driver_node` is a drop-in hardware replacement for one robot's slice of
`unicycle_fleet_node`.** It honours the exact same topics, types and frames, and
adds `/robot_i/imu/data_raw`, `/robot_i/imu/mag`, `/robot_i/voltage`.
Consequence: **the VM side (coordinator, controllers, planner, ZMQ) does not
change at all.** You are simply swapping the simulated fleet for four real Pis.

There is **no ZMQ bridge at the robot boundary** — ZMQ stays internal to the
planning layer on the VM. The robot boundary is pure ROS2/DDS over WiFi, exactly
as in your Platform 2 co-simulation (where the RPi ran `robot_sim_node`). You're
replacing `robot_sim_node` with `driver_node`.

---

## 4. Hardware architecture

```
                 ┌──────────────────────────── Ubuntu VM (laptop) ────────────────────────────┐
                 │  coordinator_node  ──ZMQ──  controller_node ×4   (the hybrid MPC, unchanged) │
                 │            └──────────── formation_planner_node (ROS2) ───────────┘          │
                 │   publishes /robot_i/cmd_vel   •   subscribes /robot_i/odom                   │
                 └───────────────────────────────────┬──────────────────────────────────────────┘
                                                      │  ROS2 / DDS over WiFi (one LAN, ROS_DOMAIN_ID shared)
        ┌──────────────────────┬──────────────────────┼──────────────────────┬──────────────────────┐
        ▼                      ▼                      ▼                      ▼                      
  ┌───────────┐          ┌───────────┐          ┌───────────┐          ┌───────────┐
  │  RPi4 #1  │          │  RPi4 #2  │          │  RPi4 #3  │          │  RPi4 #4  │     (hostname yahboom1..4)
  │ driver_no │          │ driver_no │          │ driver_no │          │ driver_no │     ns: /robot_1 .. /robot_4
  └─────┬─────┘          └─────┬─────┘          └─────┬─────┘          └─────┬─────┘
        │ USB serial (/dev/myserial, 115200)                                  
  ┌─────▼─────┐
  │ ROS Ctrl  │  STM32F103RCT6 + factory firmware (Rosmaster serial protocol)
  │ Board V3  │  • set_car_motion(v,0,ω) → closed-loop wheel PID (app_pid.c, on-chip)
  │           │  • auto-reports encoders + IMU + voltage
  └─────┬─────┘
        │ M2, M4 motor ports
  ┌─────▼─────┐
  │ 2× 520    │  L-type 520 encoder metal DC motors + caster  (differential drive)
  │ motors    │  12 V battery -> board T-port; board provides 5 V to the Pi
  └───────────┘
```

Per robot: motors on **M2 (left)** and **M4 (right)** as Yahboom advised;
board MicroUSB → Pi USB; 12 V battery → board T-connector → board's 5 V out → Pi.

---

## 5. Software architecture (per robot)

```
/robot_i/cmd_vel (v,ω) ─┐
                        ▼
            ┌───────────────────────────── driver_node (RPi i) ─────────────────────────────┐
            │  watchdog: stop if cmd_vel stale > cmd_timeout_s                                │
            │                                                                                 │
            │  actuate:  set_car_motion(v_cmd, 0, ω_cmd)                                       │
            │       → firmware Fourwheel_Ctrl() mixes (v,ω) into L/R wheel targets             │
            │       → firmware app_pid.c closes the wheel-velocity loop on-chip, 10ms tick     │
            │       → only M2 (L) / M4 (R) have real motors+encoders wired; harmless on M1/M3  │
            │                                                                                 │
            │  read encoders (m1..m4) ──► ODOMETRY ONLY (firmware reports velocity, not pose): │
            │     d=(d_L+d_R)/2 ; dθ=(d_R-d_L)/b ; x,y,θ ← integrate (shared frame)           │
            │                                                                                 │
            │  publish: /robot_i/odom  /imu/data_raw  /imu/mag  /voltage  /diagnostics        │
            └─────────────────────────────────────────────────────────────────────────────────┘
                        │
                        ▼
            /robot_i/odom (x,y,θ,v,ω) ──► back to formation_planner_node on the VM
```

A single node owns the serial port because `Rosmaster` opens the device
exclusively — **only one process per robot may hold the `Rosmaster` instance.**
All actuation and all sensor reads go through `driver_node`.

### Control path — corrected: the firmware closes the wheel-velocity loop

**This was wrong in an earlier draft of this document and of `driver_node.py`,
and is worth being explicit about the correction.** The factory firmware
exposes two different motor-command paths over the serial protocol
(`protocol.c`):

| Command (`FUNC_*`) | Rosmaster_Lib call | What it does |
|---|---|---|
| `FUNC_MOTOR` (0x10) | `set_motor(s1,s2,s3,s4)` | `Motion_Set_Pwm()` — **open-loop** PWM straight to the H-bridges. The line that would route this through the PID (`Motion_Set_Speed(...)`) is commented out in firmware, with the comment "未使用编码器" (encoder not used) directly above the case. By firmware design, this path ignores the encoders entirely. |
| `FUNC_MOTION` (0x12) | `set_car_motion(v_x, v_y, v_z)` | `Motion_Ctrl()` → `{car_type}_Ctrl()` → `Motion_Set_Speed()` → `PID_Calc_Motor()`, run every 10 ms from `Motion_Handle()`. This **is** a real closed-loop wheel-velocity PID, executing on the STM32, using live encoder feedback (`app_pid.c`). |

The original design picked `set_motor()` — the one command explicitly built
to *bypass* the encoder/PID — and then re-implemented a wheel-velocity PI
loop on the Pi on top of it, duplicating work the firmware already does
properly. The fix: command the robot with **`set_car_motion(v, 0, ω)`**,
which takes m/s and rad/s directly, and let the firmware's own PID do the
wheel-velocity regulation. No PI loop, feedforward term, or anti-windup logic
remains on the Pi side.

`set_car_motion` dispatches its kinematics by `car_type` (`Motion_Ctrl()` in
`app_motion.c` switches on it). None of the five built-in types is a literal
2WD-plus-caster robot, but `CAR_FOURWHEEL` (`car_type=4`) is the right fit:
`Fourwheel_Ctrl()` computes one left-side and one right-side wheel-speed
target from `(V_x, V_z)` — true differential-drive mixing — and applies that
target identically across all 4 PID channels via `Motion_Set_Speed()`. With
only M2 and M4 physically wired, the other two PID channels just run against
an always-zero encoder and drive nothing — harmless. `driver_node` sets
`car_type=4` via `Rosmaster(car_type=4, ...)` / `set_car_type(4)` at startup.

The firmware's default wheel-PID gains (`app_pid.h`: `Kp=0.8, Ki=0.06,
Kd=0.5`) are usually a reasonable starting point. If wheel tracking is
sluggish or oscillatory once you can scope it (Phase 3 of the deployment
guide), `driver_node` can push new gains via `set_pid_param(kp, ki, kd,
forever=False)` — exposed as the `push_pid_gains` / `wheel_pid_*` ROS2
parameters. `forever=True` burns them to flash (slow; persists across
reboot); leave it `False` while tuning.

### Odometry math (unchanged) — the firmware still does not provide pose

Checked directly in firmware: `car_data_t` holds only `{Vx, Vy, Vz}` (body
velocities); the Python library's `get_motion_data()` parses the same
`FUNC_REPORT_SPEED` velocity triple. There is no `x/y/θ` integration
anywhere in the firmware or `Rosmaster_Lib`. The reported IMU yaw
(`FUNC_REPORT_IMU_ATT`) is a real attitude estimate but not a pose solution.
So `driver_node` still integrates pose itself, from raw encoder ticks,
midpoint integration:

```
d_L = Δticks_L · meters_per_tick ;  d_R = Δticks_R · meters_per_tick
d   = (d_L + d_R)/2 ;  dθ = (d_R - d_L)/b
x  += d·cos(θ + dθ/2) ;  y += d·sin(θ + dθ/2) ;  θ = wrap(θ + dθ)
```

Two calibrated scalars are needed for this: `meters_per_tick` and
`wheel_separation_m (b)`. `max_wheel_speed_ms` is now informational only —
it no longer feeds a feedforward gain (there is no Pi-side wheel loop to feed
one into); it's still useful as a measured top-speed reference for the
`v_max`/`w_max` sanity clamps on incoming `cmd_vel`.

---

## 6. The one real-world gap you must plan for: a shared global frame

This is the biggest difference between your perfect simulator and physical
hardware, and it deserves to be called out plainly.

* In simulation, all robots live in one global `odom` frame and start exactly at
  `r0_single[i]`. The MPC closes the loop on those global positions.
* On hardware, **each robot's wheel odometry starts at `(0,0,0)` in its own
  frame and drifts independently.** There is no shared reference and no loop
  closure between robots.

Mitigations, in increasing order of fidelity:

1. **Initialize each odom to `r0_single[i]`** (what `driver_node` does via
   `initial_x/y/θ` and the launch file). Physically place and orient each robot
   at its start pose. This makes all four odometries coincide *at t=0*. Good for
   short runs; **drift grows with distance/time and rotations.** This is the
   pragmatic starting point.
2. **EKF fusion of wheel odom + IMU yaw** (Section 7) — slows heading drift, the
   dominant error for differential robots. Still no inter-robot reference.
3. **Global localization** — overhead camera + ArUco/AprilTag, UWB anchors, or
   motion capture publishing each robot's pose in a common `map` frame. This is
   the proper fix for sustained formation accuracy and is the recommended next
   hardware step once the basic loop works. The MPC then reads global pose
   instead of (or fused with) wheel odom.

Also note your `r0_single` spans ~11 m. For a small indoor space, scale the
start positions and `d_safe`/formation parameters down in `consensus_config.py`
(and the launch `R0_SINGLE` table) to fit your arena.

---

## 7. Optional accuracy enhancement: EKF + IMU

`config/ekf.yaml` provides a `robot_localization` EKF + `imu_filter_madgwick`
setup that fuses wheel odometry with the ICM20948 yaw rate to reduce heading
drift. Run them inside each `robot_i` namespace; the EKF then publishes the fused
`/robot_i/odometry/filtered` and owns the `odom→base_link` TF (set
`publish_tf:=false` on `driver_node`). Point the planner at the filtered topic.
Verify IMU units/scaling first (see the note in `driver_node._publish_imu`).

---

## 8. Files in `mpc_robot_bringup`

| File | Role |
|---|---|
| `mpc_robot_bringup/driver_node.py` | The hardware interface node (the deliverable's core). Drop-in for one robot of `unicycle_fleet_node`. |
| `mpc_robot_bringup/calibrate_node.py` | Measures `meters_per_tick`, `wheel_separation_m`, `max_wheel_speed_ms`; verifies motor/encoder signs. |
| `launch/robot_bringup.launch.py` | Brings up one robot: namespace `robot_<id>`, loads params, sets shared-frame start pose from `r0_single`. |
| `config/robot_params.yaml` | Per-robot calibration parameters (copy to `robot1..4.yaml`). |
| `config/ekf.yaml` | Optional EKF + IMU-filter fusion config. |
| `package.xml`, `setup.py`, `setup.cfg` | ament_python packaging. |

See `DEPLOYMENT_GUIDE.md` for the step-by-step bring-up.