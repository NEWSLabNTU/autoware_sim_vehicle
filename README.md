# autoware_sim_vehicle

The Autoware side of a simulated car, the same on every simulator
([scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge) roadmap 051).
The car itself is described once, in [sim_vehicle_spec](https://github.com/NEWSLabNTU/sim_vehicle_spec)
(a submodule under `deps/`); this repository holds what Autoware needs of it, the Autoware
profile a simulator's vehicle bridge runs, and the check that a vehicle bridge gives Autoware
what it needs.

| Package | What |
|---|---|
| `cars/<name>/` | Each car's Autoware packages, generated from its spec by `autoware_gen`: `<name>_description`, `<name>_launch`, `<name>_sensor_kit_description`, `<name>_sensor_kit_launch` ("The car packages") |
| `individual_params` | Every car's sensor kit calibration, where Autoware looks it up (`config/default/<name>_sensor_kit/`) |
| `autoware_sim_profile` | Autoware 1.5 set up for a simulated car, with no vehicle interface: `launch/autoware.launch.xml` ("The profile") |
| `autoware_sim_check` | `vehicle_check`: a car's vehicle side against its spec, with no Autoware and no scenario ([docs/vehicle-contract.md](docs/vehicle-contract.md)) |
| `autoware_sim_pilot` | `auto_drive`: route and engage Autoware over its API and wait for arrival, from a poses file (`config/poses/<town>.yaml`); `capture_poses`: write those files from RViz. For driving to a goal with no scenario runner |

## Build

```bash
git submodule update --init      # deps/sim_vehicle_spec
just build                       # colcon, against /opt/autoware/1.5.0 (AUTOWARE_SETUP)
just generate                    # write every car's packages from its spec
just check-generated             # fail when a car's packages are stale, or its pedal maps
                                 # were measured on other physics than the spec's
just check                       # check-generated, the tests, and the profile's copies of
                                 # autoware_launch's components still derive from upstream's
```

## The car packages

`python3 -m autoware_gen` (`just generate`) reads every spec under
`deps/sim_vehicle_spec/vehicles/` and writes, for a spec named `<name>`:

| file | from |
|---|---|
| `<name>_description/config/vehicle_info.param.yaml` | the spec's `geometry`, as is |
| `<name>_description/config/mirror.param.yaml` | the spec's `mirror` |
| `<name>_description/config/spec_hash.yaml` | `spec_hash` and `name`: what the vehicle agent reports when it registers with a scenario's relay |
| `<name>_description/config/accel_map.csv`, `brake_map.csv`, `pedal_maps.yaml` | measured, not generated ("The pedal maps") |
| `<name>_description/urdf/vehicle.xacro` | template: `base_link` and an RViz box from `vehicle_info` |
| `<name>_launch/launch/vehicle_interface.launch.xml` | template: empty. The simulator's vehicle bridge is the interface |
| `<name>_sensor_kit_description/config/sensors_calibration.yaml` | the spec's `sensor_kit.mount` (base_link -> sensor_kit_base_link) |
| `<name>_sensor_kit_description/config/sensor_kit_calibration.yaml` | each sensor's `mount`, under its calibration key |
| `<name>_sensor_kit_description/urdf/sensor_kit.xacro` | a link per sensor `link`, and a child `frame` when it differs (the optical rotation for a camera) |
| `<name>_sensor_kit_launch/` | templates: the sensing pipeline (lidar concatenation and preprocessing, IMU corrector, GNSS poser, vehicle velocity converter) |
| `individual_params/config/default/<name>_sensor_kit/` | the two calibration files again |

The templates are `autoware_gen/templates/` (`@CAR@` and `@KIT@` the package names). Every
computed file starts with a header naming its spec and `spec_hash`. Autoware selects a car
with `vehicle_model:=<name> sensor_model:=<name>_sensor_kit`. `just check-generated` fails,
naming the file, when a generated file differs from what the spec gives.

### The pedal maps

`accel_map.csv` and `brake_map.csv` are Autoware's pedal maps for the car, read by
acb_bridge (and by Autoware's `raw_vehicle_cmd_converter`). They are **measured** on CARLA
with the spec's physics (acb's `scripts/build_longitudinal_maps.py`), not generated.
`pedal_maps.yaml` beside them records what they were measured on:

```yaml
measured_on: spec         # or blueprint: CARLA's own physics, for a spec without carla.physics
physics_hash: 33635ad5... # sim_vehicle_spec's physics_hash of the spec they were measured on
basis: "vehicle.tesla.model3, CARLA tick 0.050 s, ..."
```

`just check-generated` fails when either map is missing, when `pedal_maps.yaml` is missing,
when the spec has `carla.physics` and the recorded `physics_hash` is not the spec's (re-measure
them), or when the spec has no `carla.physics` and the maps were not measured on the
blueprint. `physics_hash` covers only what drives the car, so moving a sensor does not
invalidate the maps.

## The profile

`autoware_sim_profile/launch/autoware.launch.xml` is Autoware 1.5 with the settings a
simulated car needs, and **no vehicle interface** (`launch_vehicle_interface:=false`). It
needs no simulator and no scenario runner. A simulator's vehicle bridge includes it and starts
its own interface beside it: acb's `carla_simulator.launch.xml` adds `acb_bridge`; aib's
`isaac_simulator.launch.xml` adds nothing.

Required: `map_path`, `vehicle_model`, `sensor_model`. The arguments a simulation changes:

| argument | default | |
|---|---|---|
| `sim_localization` | `true` | simulation-tuned localization parameters (`sim_loc_config_path`) |
| `sim_perception` | `true` | a selectable lidar model, fusion-only traffic lights |
| `sim_system` | `true` | simulation component-state topics, MRM handler, diagnostic graph (`sim_system_topics_path`, `sim_mrm_handler_path`) |
| `sim_control` | `true` | MPC with steering offset learning off (`sim_lat_controller_param_path`) |
| `sim_planning` | `true` | planning with a selectable `max_velocity` |
| `sim_clock_guard` | `true` | start `clock_guard` (below) |
| `max_velocity` | `4.17` | Autoware's `max_vel`, m/s (upstream's); needs `sim_planning` |
| `lidar_detection_model` | `clustering` | or `centerpoint` (TensorRT) |
| `traffic_light_fusion_only` | `true` | signals from the vehicle bridge (V2X), no camera recognition |
| `use_multi_channel_tracker_merger`, `occupancy_grid_map_method`, `perception`, `publish_ground_truth_objects` | | perception choices; see the launch file |
| `extra_launch` | empty | a launch file of your own to include beside the stack, in its domain (scenario-world-bridge's vehicle agent is one); it receives `map_path` and `use_sim_time` |
| `gpu` | empty | `CUDA_VISIBLE_DEVICES` for the stack |
| `data_path` | `$HOME/autoware_data` | must be writable; fill it once with `ros2 run autoware_sim_profile setup_autoware_data` |
| `autoware_launch` | autoware_launch's | the `autoware.launch.xml` the profile includes (a fork's, for example) |

A `sim_*` switch set to `false` hands that component back to `autoware_launch`, unchanged.

**`clock_guard`** (`autoware_sim_profile`'s node, started when `sim_clock_guard:=true`) ends
the stack when `/clock` goes back. An episode (a simulator world, from its load to the next)
is the stack's lifetime: Autoware cannot follow time back, so the node exits with status 3,
logging both times, the moment `/clock` steps back by more than its `tolerance` parameter
(seconds, default 0). It is declared `on_exit="shutdown"`, so its exit ends every node of the
launch, and whatever started the stack restarts it on the new episode. A vehicle bridge should
end the stack first (acb_bridge exits on CARLA's episode change before it publishes anything
of the new one); `clock_guard` is the backstop for every simulator.

**`check_launch_args`** refuses the arguments roadmap 051 renamed. ROS launch silently ignores
an argument no file declares, so `carla_control:=false` would be dropped and the default used.
Every wrapper that hands its arguments to the profile runs it first:

```bash
ros2 run autoware_sim_profile check_launch_args name:=value ...   # exit 0, or 2 naming each
```

It refuses `carla_control`, `carla_localization`, `carla_perception`, `carla_planning`,
`carla_system` and the `carla_*_path` arguments (now `sim_*`), and the values `acb_vehicle`,
`aib_vehicle`, `acb_sensor_kit`, `aib_sensor_kit` (now `tesla_model3` and
`tesla_model3_sensor_kit`).

## vehicle_check

The contract every vehicle bridge must meet (topics, types, rates on the messages' own
stamps, frames, `/clock`, `/robot_description`) and a dynamics run (acceleration and braking
steps judged against 0.35 m/s^2, a steering sweep against the bicycle model), against the
car's spec, with no Autoware and no SSv2:

```bash
# a vehicle side up (acb_bridge with a hero in CARLA, aib's runner in Isaac), then:
play_launch launch autoware_sim_check vehicle_check.launch.xml simulator:=carla out:=report.json
```

The launch ends when the check does; the report is JSON. The full contract, the checks, the
arguments, the report and how to build a vehicle side for another simulator:
[docs/vehicle-contract.md](docs/vehicle-contract.md).

## Driving to a goal without a scenario

```bash
ros2 run autoware_sim_pilot auto_drive --ros-args -p poses_file:=<poses.yaml>   # beside a running stack
```

`auto_drive` waits for Autoware's services and for localization (from GNSS), sets a route to
the file's `goal_pose`, engages, and exits when the car arrives. Parameters: `poses_file`,
`timeout` (600 s), `stabilize_seconds` (15), `route_timeout` (120), `spawn_timeout` (1800),
`localization_gap_limit_m` (5).
