# Deployment Guide — `mpc_robot_bringup`

End-to-end bring-up: from one robot rolling under `cmd_vel` to four robots
running your distributed MPC formation controller. Work through Phases 1–4 on a
**single robot first**, verify, then replicate to the other three.

Conventions: the VM is your Ubuntu laptop running the MPC; `yahboom1..4` are the
four Raspberry Pi 4s (`username = pi`). All machines run ROS2 Humble.

---

## Phase 0 — Prerequisites (each RPi4, once)

You have already (per your notes): assembled the robot, wired M2/M4, connected
the board's MicroUSB to the Pi, bound the serial device, and installed the
`Rosmaster_Lib` driver (`sudo python3 setup.py install` from `py_install_V3.3.9`).

Confirm the basics on `yahboom1`:

```bash
# 1. Serial device exists (you bound it in Yahboom Chapter 2):
ls -l /dev/myserial

# 2. Rosmaster_Lib imports:
python3 -c "from Rosmaster_Lib import Rosmaster; print('lib OK')"

# 3. ROS2 Humble sourced:
source /opt/ros/humble/setup.bash
ros2 --version
```

If `/dev/myserial` does not exist, create a stable udev symlink so it survives
reboots and USB re-plugs (replace idVendor/idProduct from `lsusb`; the board's
USB-serial is a CH340, typically `1a86:7523`):

```bash
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0666", SYMLINK+="myserial"' \
  | sudo tee /etc/udev/rules.d/99-myserial.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add the user to the serial group once (then log out/in):
```bash
sudo usermod -aG dialout pi
```

---

## Phase 1 — Install the package (each RPi4)

```bash
# Create a workspace and drop the package in:
mkdir -p ~/mpc_ws/src
# copy the mpc_robot_bringup folder into ~/mpc_ws/src/  (scp, git, or USB)

cd ~/mpc_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select mpc_robot_bringup
source install/setup.bash
```

Sanity check that the executables registered:
```bash
ros2 pkg executables mpc_robot_bringup
# -> mpc_robot_bringup calibrate_node
#    mpc_robot_bringup driver_node
```

---

## Phase 2 — Calibrate (each robot, once)

Motors and gearboxes vary per unit, so calibrate **each** robot and save its
numbers in its own `robotN.yaml`. Run one routine at a time.

> Put the robot **on blocks** for `sign` and `maxspeed`; give it clear floor for
> `straight` and `spin`. Keep a hand on the power switch.

**2.1 Check motor/encoder signs** (robot on blocks):
```bash
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=sign -p pwm:=30
```
Each driven wheel must (a) roll the robot **forward** and (b) print a
**positive** `dticks`. If a wheel rolls backward → that's a real wiring
issue: swap that motor's two leads (or its connector orientation) at the
board. **Note:** this is now a *physical* fix, not a config one — `set_motor`
(what this diagnostic uses) has a per-port sign you can flip in
`calibrate_node`'s own params to confirm the direction, but `driver_node`'s
runtime path (`set_car_motion`) has a single fixed sign convention for the
whole robot (positive v = forward, positive ω = one fixed turn direction)
with no per-wheel software override. If it rolls forward but its delta is
negative → flip that wheel's `*_encoder_sign` in `robot_params.yaml` (this
one *is* still a `driver_node` parameter, since it only affects odometry).

**2.2 `meters_per_tick`** (clear floor, ~1–2 m):
```bash
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=straight -p pwm:=30 -p seconds:=3.0
```
Tape-measure the real distance travelled. Then
`meters_per_tick = measured_distance_m / avg_ticks` (the node prints `avg_ticks`
and the arithmetic for a 1.00 m drive).

**2.3 `max_wheel_speed_ms`** (on blocks; uses your `meters_per_tick`):
```bash
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=maxspeed \
  -p seconds:=3.0 -p meters_per_tick:=<your_value>
```
Use the printed wheel speed (the slower wheel) as `max_wheel_speed_ms`.

**2.4 `wheel_separation_m` (b)** (clear floor; mark a reference line):
```bash
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=spin -p pwm:=30 \
  -p seconds:=4.0 -p meters_per_tick:=<your_value> -p wheel_separation_m:=0.170
```
Watch the robot do roughly one turn. If it physically completed 360° but the
printed `theta` says, e.g., 350°, correct: `b_new = b_old · (350/360)`. Re-run to
confirm `theta ≈ 360°`.

**2.5 Save** the four numbers into a per-robot file:
```bash
cp ~/mpc_ws/src/mpc_robot_bringup/config/robot_params.yaml \
   ~/mpc_ws/src/mpc_robot_bringup/config/robot1.yaml
# edit robot1.yaml: meters_per_tick, wheel_separation_m, max_wheel_speed_ms, signs
colcon build --packages-select mpc_robot_bringup && source install/setup.bash
```

---

## Phase 3 — Single-robot bring-up and smoke test

Launch robot 1 (uses `r0_single[1]` as the shared-frame start pose):

```bash
ros2 launch mpc_robot_bringup robot_bringup.launch.py \
  robot_id:=1 params_file:=$HOME/mpc_ws/install/mpc_robot_bringup/share/mpc_robot_bringup/config/robot1.yaml
```

In a second terminal, confirm the contract:
```bash
ros2 topic list | grep robot_1
# /robot_1/cmd_vel  /robot_1/odom  /robot_1/imu/data_raw  /robot_1/imu/mag
# /robot_1/voltage  /robot_1/diagnostics

ros2 topic echo /robot_1/odom --once     # pose should read your initial_x/y/θ
ros2 topic echo /robot_1/voltage --once  # ~11–12.6 V on a charged 3S
```

**Open-loop teleop test** (robot on floor, clear space):
```bash
# drive forward 0.15 m/s for ~2 s, then stop:
ros2 topic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
# Ctrl-C -> watchdog stops the robot within cmd_timeout_s
```
Verify: robot drives straight; `/robot_1/odom` x advances ≈ the real distance;
θ stays near its start; stopping the publisher stops the robot (watchdog).

**Turn test:** publish `angular.z: 0.5` and confirm odom θ increases in the
correct direction (CCW positive). If x drifts during a pure turn or the turn rate
is off, revisit `b` (Phase 2.4). If it veers when going straight, your two wheels
aren't matched — check signs and consider a per-wheel `meters_per_tick`.

**Tune the wheel loop** if tracking is sluggish or oscillates. The
velocity loop now runs **on the firmware** (`set_car_motion` → on-chip PID,
not a Pi-side loop), so tuning means changing the gains the firmware uses,
not anything in `driver_node`'s control code. Set in `robot1.yaml`:
```yaml
push_pid_gains: true
wheel_pid_kp: 0.8   # raise until response is brisk
wheel_pid_ki: 0.06  # raise to kill steady-state error; back off if it oscillates
wheel_pid_kd: 0.5
wheel_pid_forever: false   # keep false while tuning -- true burns to flash (slow)
```
Relaunch after each change (gains are pushed once at startup via
`set_pid_param()`). Once you're happy, you can leave `push_pid_gains: false`
to just use the firmware defaults, or set `wheel_pid_forever: true` for one
launch to persist your tuned values to flash.

---

## Phase 4 — Four robots + the MPC

### 4.1 Network (all machines on one WiFi LAN)

Every machine (VM + 4 Pis) must share the same `ROS_DOMAIN_ID` and discover each
other over multicast/unicast DDS:

```bash
# put in ~/.bashrc on the VM and on each Pi (same number everywhere):
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

Verify cross-machine discovery: launch robot 1, then on the VM run
`ros2 topic list` and confirm `/robot_1/...` appears. If not, see Troubleshooting
(WiFi multicast is the usual culprit; a unicast discovery profile fixes it).

> WiFi tip from your own notes: align QoS across publishers/subscribers. This
> node uses RELIABLE depth-10 to match the sim fleet. If you see odom stalls over
> WiFi, consider switching odom to SENSOR_DATA (BEST_EFFORT) on both ends.

### 4.2 Place and orient the robots

Physically place each robot at its `r0_single` position and orient it to its
start heading (default θ=0, all facing +x). The launch file injects these as
`initial_x/y/θ` so all four odometries share one frame at t=0. (Scale
`r0_single` in `consensus_config.py` and the launch `R0_SINGLE` table if your
arena is smaller than ~11 m.)

### 4.3 Launch each robot (on its own Pi)

```bash
# yahboom1:
ros2 launch mpc_robot_bringup robot_bringup.launch.py robot_id:=1 \
  params_file:=.../config/robot1.yaml
# yahboom2:
ros2 launch mpc_robot_bringup robot_bringup.launch.py robot_id:=2 \
  params_file:=.../config/robot2.yaml
# yahboom3 -> robot_id:=3 ... ; yahboom4 -> robot_id:=4 ...
```

From the VM, confirm all four:
```bash
for i in 1 2 3 4; do ros2 topic echo /robot_$i/odom --once; done
```

### 4.4 Start the MPC on the VM (unchanged)

Bring up your existing planning layer exactly as in Platform 2 — coordinator,
the four controllers, and `formation_planner_node` — with the same
`ROS_DOMAIN_ID`. The planner will subscribe `/robot_i/odom` and publish
`/robot_i/cmd_vel`; the real robots now stand in for `unicycle_fleet_node`. No
VM-side code changes are required.

Watch it converge:
```bash
ros2 topic echo /robot_1/cmd_vel        # planner commands arriving
ros2 run rqt_plot rqt_plot              # plot /robot_i/odom poses if desired
```

> First full run: keep speeds low (cap the planner's `k_speed` / `u_max`), test
> in an open area, and be ready to cut power. Wheel-odometry drift means the
> physical formation will be looser than in sim — see ARCHITECTURE.md §6 and add
> global localization when you need sustained accuracy.

---

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `serial.serialutil.SerialException` on start | Wrong/missing `/dev/myserial`; user not in `dialout`; another process holds the port (only ONE node per board). |
| Robot doesn't move on `cmd_vel` | Motors on M1/M3 not M2/M4 (`left/right_encoder_index` wrong, or `car_type` not 4); `cmd_timeout_s` too small for your publish rate. |
| Drives backward / turns wrong way | Diagnose with `sign` routine (Phase 2.1): a wired-backward wheel needs a physical lead swap (no software motor_sign on the runtime path); an inverted reading needs `*_encoder_sign` flipped in `robot_params.yaml`. |
| Odom x goes negative when driving forward | Flip `*_encoder_sign`. |
| Odom distance wrong by a constant factor | Recalibrate `meters_per_tick` (Phase 2.2). |
| Turn angle off / x drifts during pure spin | Recalibrate `wheel_separation_m` (Phase 2.4). |
| Veers when commanded straight | Wheels not matched: check signs; try per-wheel `meters_per_tick`; check tire/caster. |
| Sluggish or oscillating speed tracking | Tune firmware wheel PID gains: `push_pid_gains: true` + `wheel_pid_kp/ki/kd` (Phase 3). This now tunes the on-chip PID via `set_pid_param`, not a Pi-side loop. |
| VM can't see `/robot_i/...` topics | `ROS_DOMAIN_ID` mismatch; `ROS_LOCALHOST_ONLY=1`; WiFi blocks multicast → use a CycloneDDS/FastDDS unicast peer profile listing all 5 IPs. |
| Odom updates stall/jitter over WiFi | Switch odom QoS to BEST_EFFORT on both ends; reduce other WiFi traffic. |
| Low-voltage warnings | 3S LiPo low (<10 V); recharge. Your voltage alarm should also trigger. |

---

## Quick command reference

```bash
# build
cd ~/mpc_ws && colcon build --packages-select mpc_robot_bringup && source install/setup.bash

# calibrate (one routine at a time)
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=sign -p pwm:=30
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=straight -p pwm:=30 -p seconds:=3.0
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=maxspeed -p seconds:=3.0 -p meters_per_tick:=<v>
ros2 run mpc_robot_bringup calibrate_node --ros-args -p routine:=spin -p pwm:=30 -p meters_per_tick:=<v>

# bring up robot i
ros2 launch mpc_robot_bringup robot_bringup.launch.py robot_id:=i params_file:=.../robotI.yaml

# manual drive / stop
ros2 topic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist '{linear: {x: 0.15}, angular: {z: 0.0}}'
```