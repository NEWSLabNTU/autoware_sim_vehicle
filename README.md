# autoware_sim_vehicle

The Autoware side of a simulated car, the same on every simulator
([scenario-world-bridge](https://github.com/NEWSLabNTU/scenario-world-bridge) roadmap 051).

| Package | What |
|---|---|
| `autoware_sim_profile` | Autoware 1.5 tuned for simulation, with no vehicle interface: `launch/autoware.launch.xml`. A simulator's vehicle bridge includes it and starts its own interface beside it ([autoware_carla_bridge](https://github.com/NEWSLabNTU/autoware_carla_bridge) on CARLA, [autoware_isaac_bridge](https://github.com/NEWSLabNTU/autoware_isaac_bridge) on Isaac Sim) |

The profile needs no simulator and no scenario runner. Its simulation settings are switchable
(`sim_localization`, `sim_perception`, `sim_system`, `sim_control`, `sim_planning`; false
hands the component back to your `autoware.launch.xml`), and `max_velocity`,
`lidar_detection_model` and `traffic_light_fusion_only` choose among them.

```bash
just build    # against /opt/autoware/1.5.0 (AUTOWARE_SETUP)
ros2 run autoware_sim_profile setup_autoware_data   # once: a writable data_path for TensorRT
just check    # the component copies still derive from autoware_launch's
```

Per-car packages generated from the vehicle spec, and `vehicle_check` (the contract and the
dynamics run every simulator's bridge is checked against), come with roadmap 051 K2-K3.
