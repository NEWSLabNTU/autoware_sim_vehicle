# The vehicle-side contract and `vehicle_check`

What a simulator's **vehicle side** (its vehicle bridge: acb_bridge on CARLA, aib's runner on
Isaac Sim) gives Autoware, and how `vehicle_check` (package `autoware_sim_check`) checks it
with no Autoware and no scenario runner. This is the I5 contract of scenario-world-bridge's
design ([vehicle-side](https://github.com/NEWSLabNTU/scenario-world-bridge/blob/main/docs/design/vehicle-side.md),
"The I5 contract"). A new simulator is compatible with the shared Autoware packages when
`vehicle_check` passes against its vehicle side.

Everything is checked against the car's **vehicle spec** (`sim_vehicle_spec`,
`vehicles/<vehicle_model>.yaml`): its sensors, topics, frames, rates and wheel base. The
tables below are for the shipped `tesla_model3`; another spec gives other sensor rows.

## What a vehicle side provides

All on one ROS domain, all with simulation time.

### `/clock`

`rosgraph_msgs/msg/Clock`, the simulation time. Every stamp below is on this clock.

- The vehicle side publishes it (it is the simulator's time), reliable or best effort
  (`vehicle_check` subscribes best effort, which matches either; Autoware's nodes take it
  reliable, so reliable is the safe choice). Publish it at least as often as the fastest
  sensor.
- Time is `nanos(elapsed) + BASE_NS` (1000 s; see swb_world_protocol's `sim_time`), monotonic
  within one simulator episode. When the episode changes (a world load, a simulator restart)
  the vehicle side must exit before it publishes anything of the new episode, so the stack
  ends; the profile's `clock_guard` is the backstop when it does not ("The profile" in the
  README).

### The vehicle interface: published

| topic | type | `vehicle_check` requires | Autoware reads |
|---|---|---|---|
| `/vehicle/status/velocity_status` | `autoware_vehicle_msgs/msg/VelocityReport` | at least 10 Hz on its stamps, `header.frame_id` `base_link` | `longitudinal_velocity` (m/s), `heading_rate` (yaw rate, rad/s, positive turning left) |
| `/vehicle/status/steering_status` | `autoware_vehicle_msgs/msg/SteeringReport` | at least 10 Hz on its `stamp` | `steering_tire_angle` (rad, the bicycle-model tire angle, positive left) |
| `/vehicle/status/gear_status` | `autoware_vehicle_msgs/msg/GearReport` | at least 5 Hz on its `stamp` | `report` (`DRIVE` when driving forward) |
| `/vehicle/status/control_mode` | `autoware_vehicle_msgs/msg/ControlModeReport` | at least 5 Hz on its `stamp` | `mode` (`AUTONOMOUS` for Autoware to drive) |
| `/vehicle/status/turn_indicators_status` | `autoware_vehicle_msgs/msg/TurnIndicatorsReport` | not checked | |
| `/vehicle/status/hazard_lights_status` | `autoware_vehicle_msgs/msg/HazardLightsReport` | not checked | |

The design asks for the status topics at 20 Hz or more; `vehicle_check` enforces the lower
floors above. Publish `velocity_status` at 20 Hz or more in any case: the dynamics run judges
each braking step on the samples between 0.5 and 5.5 m/s, and a hard braking step spends
about half a second there.

### The vehicle interface: taken

| topic | type | |
|---|---|---|
| `/control/command/control_cmd` | `autoware_control_msgs/msg/Control` | `longitudinal.acceleration` (m/s^2), realised against the spec's physics; `longitudinal.velocity` (the target); `lateral.steering_tire_angle` (rad) |
| `/control/command/gear_cmd` | `autoware_vehicle_msgs/msg/GearCommand` | |
| `/control/command/turn_indicators_cmd`, `hazard_lights_cmd`, `emergency_cmd` | Autoware's | not used by `vehicle_check` |

Both are published reliable with depth 10, as Autoware's controller does; subscribe reliable
or best effort.

`vehicle_check` sends only `control_cmd` and `gear_cmd` (`DRIVE`), with no engage and no
control-mode request, and leaves `longitudinal.is_defined_acceleration` false. A vehicle side
must therefore act on `control_cmd` as it arrives and use `acceleration` whatever that flag
says (acb_bridge does).

### Sensors

One topic per spec sensor that the vehicle side spawns. Which ones depends on the simulator
(`--simulator`): `carla` expects the sensors with a `carla` block in the spec, `isaac` those
with `isaac.enabled` true (the default), `any` only the sensors that are in both sets (for
`tesla_model3`: `top`, `imu_link`, `gnss_link`; not `camera6`).

| sensor (spec) | topic | type | `header.frame_id` | at least, Hz | carla | isaac |
|---|---|---|---|---|---|---|
| `top` (lidar) | `/sensing/lidar/top/pointcloud_before_sync` | `sensor_msgs/msg/PointCloud2` | `top` | 16 | yes | yes |
| `camera6` (camera) | `/sensing/camera/camera6/image_raw` | `sensor_msgs/msg/Image` | `camera6/camera_optical_link` | 8 | yes | no |
| `imu` | `/sensing/imu/imu_link/imu_raw` | `sensor_msgs/msg/Imu` | `imu_link` | 10 | yes | yes |
| `gnss` | `/sensing/gnss/gnss_link/nav_sat_fix` | `sensor_msgs/msg/NavSatFix` | `gnss_link` | 8 | yes | yes |

In general: the topic is the sensor's `topic`, the frame its `frame`, the type follows its
`type` (lidar `PointCloud2`, camera `Image`, imu `Imu`, gnss `NavSatFix`), and the floor is
0.8 x its `rate_hz`, or 10 Hz when the spec gives no rate (the IMU above).

What Autoware's sensing pipeline needs beyond what `vehicle_check` looks at: the lidar as
Autoware's `PointXYZIRC` layout; `NavSatFix` in the projection the map's
`map_projector_info.yaml` names; a camera's `CameraInfo` on its `info_topic`. A sample carries
the time of the frame it was taken in.

### Frames

`base_link` is on the ground at the rear axle centre, x forward, y left, z up. The sensor
frames are the generated URDF's (`<car>_sensor_kit_description`), which
`robot_state_publisher` publishes from `/robot_description` as Autoware's vehicle launch
starts it. The vehicle side publishes no TF that the URDF owns.

## What `vehicle_check` checks

### The contract (`contract`)

It waits up to 120 s of wall time for `/clock` and `velocity_status`, then listens for
`--listen` seconds of simulation time (10 by default) to every topic above (the
`vehicle_check` columns and the sensors of `--simulator`), with the sensor-data QoS (best
effort, so any publisher matches). Each topic passes when:

- **its type is the contract's**: a topic the graph has with another type fails as
  `type <found>, expected <type>` (a publisher of the wrong type never delivers to the
  subscription, so the check asks the graph);
- **something arrived**: else `nothing arrived`;
- **its stamps never go back**: equal stamps are allowed;
- **its rate on its own stamps** reaches the floor: `(n - 1) / (last stamp - first stamp)`.
  Counting stamps rather than arrivals means a simulator slower than real time is not a slow
  vehicle side;
- **its frame** (`velocity_status` and the sensors) is exactly the expected `frame_id`, in
  every message;
- **its stamps stay near `/clock`**: no message's stamp is more than 1.0 s from the latest
  `/clock` at its arrival.

`/robot_description` (`std_msgs/msg/String`, transient local) must contain
`<link name="...">` for `base_link`, `sensor_kit_base_link`, and each expected sensor's
`link` and `frame`. `vehicle_check.launch.xml` starts the car's description, so a vehicle
side does not publish it.

### The dynamics (`dynamics`)

The car must stand on a flat straight road with about 150 m free ahead, or on a ground plane.
`vehicle_check` drives it through `/control/command/control_cmd` every 30 ms of wall time and
reads it back from `velocity_status` (speed, yaw rate) and `steering_status` (angle):

1. 3 s of wall time holding still: acceleration -1.5 m/s^2, target velocity 0.
2. Six longitudinal steps, in this order: accelerate +1.0, brake -1.0, accelerate +2.0,
   brake -2.5, accelerate +3.0, brake -4.0 m/s^2. An acceleration step commands its
   acceleration with a target velocity of 16 m/s and ends at 6 m/s; a braking step commands
   its deceleration with a target of 0 and ends at 0.2 m/s or below. Either ends after 20 s of
   simulation time.
3. Six steering steps, +0.05, -0.05, +0.10, -0.10, +0.15, -0.15 rad of tire angle, 2.5 s of
   simulation time each, alternating sides so the car stays near its lane, driving toward
   4 m/s by a proportional controller on the acceleration (`1.5 x (4 - v)`, within +-2
   m/s^2; target velocity 4 m/s).
4. 3 s of wall time braking at -2.0 m/s^2 to a stop.

Each step is timed from `/clock` at its start; a sample's time is its `velocity_status`
stamp minus that. Judging:

- **Acceleration and braking**: the slope of a least-squares line through speed against time,
  over the samples at least 0.8 s into the step (the actuator lags) with a speed between 0.5
  and 5.5 m/s, must be within **0.35 m/s^2** of the request (scenario-world-bridge's
  acceptance.py longitudinal limit). Fewer than 3 samples there fails the step.
- **Steering**: over the second half of the step, the mean reported tire angle must be within
  0.02 rad of the command, and the mean yaw rate within 15 % of the bicycle model's,
  `v tan(delta) / L`, with `v` the mean speed, `delta` the command and `L` the spec's
  `geometry.wheel_base`.
- A step that sees no progress for 120 s of wall time (the simulator's clock stopped) fails,
  and ends the run.

### Running it

Start the vehicle side first (acb_bridge with a hero in CARLA, aib's runner in Isaac, or your
own), then:

```bash
play_launch launch autoware_sim_check vehicle_check.launch.xml simulator:=carla out:=report.json
```

| argument | default | |
|---|---|---|
| `vehicle_model` | `tesla_model3` | the spec, and `<vehicle_model>_description` |
| `sensor_model` | `<vehicle_model>_sensor_kit` | |
| `simulator` | `any` | `carla`, `isaac` or `any`: which of the spec's sensors the vehicle side publishes |
| `what` | `all` | `contract`, `dynamics` or `all` (the contract, then the dynamics) |
| `listen` | `10.0` | seconds of simulation time the contract listens |
| `out` | `/tmp/vehicle_check.json` | the report |

The launch sets `use_sim_time` for everything, includes Autoware's
`tier4_vehicle_launch/launch/vehicle.launch.xml` with `launch_vehicle_interface:=false` (the
car's `robot_state_publisher`, so `/robot_description` and the sensor frames exist), and
starts the `vehicle_check` node with `on_exit="shutdown"`: when the check ends, whatever its
verdict, the launch ends. It needs Autoware 1.5 (for `tier4_vehicle_launch`) and this
repository's car packages built.

Without the launch (the vehicle description started some other way):

```bash
ros2 run autoware_sim_check vehicle_check [contract|dynamics|all] --vehicle-model tesla_model3 \
    [--simulator carla|isaac|any] [--listen 10] [--out report.json]
```

It prints `PASS` or `FAIL` per topic (with each problem) and per step, then
`vehicle_check: PASS` or `FAIL`, and exits 0 only when everything passed.

### The report

`--out` (the launch's `out`) is JSON:

```json
{
 "vehicle_model": "tesla_model3",
 "simulator": "carla",
 "contract": {
  "/vehicle/status/velocity_status": [],
  "/sensing/lidar/top/pointcloud_before_sync": ["...: 9.8 Hz on its stamps, expected at least 16.0"],
  "/robot_description": []
 },
 "dynamics": [
  {"step": "accel +1.0", "request": 1.0, "measured": 0.996, "error": -0.004, "pass": true},
  {"step": "steer +0.05", "request": 0.05, "steer": 0.0499, "speed": 4.0, "yaw_rate": 0.0665,
   "bicycle_yaw_rate": 0.0666, "yaw_share_off": -0.002, "pass": true}
 ]
}
```

- `contract`: every expected topic and `/robot_description`, each with its list of problems
  (empty: it passed). Present when the contract ran.
- `dynamics`: one object per step, in order. Longitudinal steps carry `measured` (m/s^2) and
  `error` (measured minus request); steering steps carry `steer` (mean reported angle, rad),
  `speed`, `yaw_rate`, `bicycle_yaw_rate` (rad/s) and `yaw_share_off` (the yaw rate's share
  off the model). A step that could not be judged has `pass: false` and `why`. A run that
  could not start is one entry, `{"step": "start", "pass": false, "why": ...}`. Present when
  the dynamics ran.

## Building a vehicle side for another simulator

The smallest thing that passes, as a checklist:

1. Publish `/clock` (reliable) from the simulator's time, `nanos(elapsed) + BASE_NS`.
2. Each tick, publish the four status topics, stamped with that tick's time:
   `velocity_status` (`frame_id` `base_link`, speed and yaw rate), `steering_status` (the
   tire angle), `gear_status` (`DRIVE`), `control_mode` (`AUTONOMOUS`). At 20 Hz or more.
3. Take `control_cmd`: realise `longitudinal.acceleration` (a car braking at or toward zero
   speed stops and stays at 0; it never rolls backwards on a brake command) and `lateral.steering_tire_angle`. With a kinematic bicycle model,
   `yaw_rate = v tan(delta) / wheel_base`, a fake passes the dynamics run exactly; a real
   simulator's physics passes when it tracks within the limits above.
4. Publish each spec sensor the simulator has on its `topic`, in its `frame`, at its rate,
   stamped with the tick it was taken in.
5. Start `vehicle_check.launch.xml` beside it with the right `simulator:=` (or a spec whose
   sensor set matches what you publish).

Beyond `vehicle_check`, for a real stack: the turn indicator and hazard light reports, the
other commands, Autoware's point layout and GNSS projection, and ending on an episode change.
