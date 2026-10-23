# autoware_sim_vehicle: the Autoware side of a simulated car -- the profile (and, from roadmap
# 051 K2, the per-car packages generated from the vehicle spec, and vehicle_check).

autoware_setup := env_var_or_default('AUTOWARE_SETUP', '/opt/autoware/1.5.0/setup.bash')

default:
    @just --list

build:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    colcon build --symlink-install --base-paths autoware_sim_profile autoware_sim_pilot

# The profile's copies of autoware_launch's components still derive from upstream's
check:
    python3 -m pytest -q test
    scripts/sync_control_component.sh --check
    scripts/sync_perception_component.sh --check
    scripts/sync_planning_component.sh --check
