# autoware_sim_vehicle: the Autoware side of a simulated car: the profile, the pilot, and each
# car's packages generated from its vehicle spec (deps/sim_vehicle_spec, a submodule).

autoware_setup := env_var_or_default('AUTOWARE_SETUP', '/opt/autoware/1.5.0/setup.bash')

default:
    @just --list

build:
    #!/usr/bin/env bash
    set -e
    source "{{autoware_setup}}"
    colcon build --symlink-install --base-paths autoware_sim_profile autoware_sim_pilot autoware_sim_check cars individual_params deps/sim_vehicle_spec

# Write every car's packages from its spec
generate:
    python3 -m autoware_gen

# Fail when a car's packages are stale, or its pedal maps were measured on other physics
check-generated:
    python3 -m autoware_gen --check

# The generated packages are current; the profile's copies of autoware_launch's components
# still derive from upstream's
check: check-generated
    python3 -m pytest -q test
    cd autoware_sim_check && python3 -m pytest -q -p no:cacheprovider test
    scripts/sync_control_component.sh --check
    scripts/sync_perception_component.sh --check
    scripts/sync_planning_component.sh --check
