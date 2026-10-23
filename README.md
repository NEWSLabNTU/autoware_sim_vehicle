# autoware_sim_vehicle

The Autoware side of a simulated car, the same on every simulator
([scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge) roadmap 051).

| Package | What |
|---|---|
| `autoware_sim_profile` | Autoware 1.5 tuned for simulation, with no vehicle interface: `launch/autoware.launch.xml`. A simulator's vehicle bridge includes it and starts its own interface beside it ([autoware_carla_bridge](https://github.com/NEWSLabNTU/autoware_carla_bridge) on CARLA, [autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge) on Isaac Sim) |
| `cars/<name>/` | Each car's Autoware packages, generated from its spec (`deps/sim_vehicle_spec/vehicles/<name>.yaml`) by `autoware_gen`: `<name>_description` (vehicle_info, mirror, the pedal maps measured on CARLA), `<name>_launch`, `<name>_sensor_kit_description` (calibration, URDF), `<name>_sensor_kit_launch` (the sensing pipeline). Autoware selects them with `vehicle_model:=<name> sensor_model:=<name>_sensor_kit` |
| `individual_params` | Every car's sensor kit calibration, where Autoware looks it up |
| `autoware_sim_pilot` | `auto_drive`: route and engage Autoware over its API and wait for arrival, from a poses file (`config/poses/<town>.yaml`); `capture_poses`: write those files from RViz. For driving to a goal with no scenario runner |

The profile needs no simulator and no scenario runner. Its simulation settings are switchable
(`sim_localization`, `sim_perception`, `sim_system`, `sim_control`, `sim_planning`; false
hands the component back to your `autoware.launch.xml`), and `max_velocity`,
`lidar_detection_model` and `traffic_light_fusion_only` choose among them.

```bash
just build    # against /opt/autoware/1.5.0 (AUTOWARE_SETUP)
ros2 run autoware_sim_profile setup_autoware_data   # once: a writable data_path for TensorRT
ros2 run autoware_sim_pilot auto_drive --ros-args -p poses_file:=<poses.yaml>   # beside a running stack
just check    # the component copies still derive from autoware_launch's
```

`just generate` writes every car's packages; `just check-generated` fails when one is stale
or its pedal maps were measured on other physics than the spec's. `vehicle_check` (the
contract and the dynamics run every simulator's bridge is checked against) comes with roadmap
051 K3.
