#!/usr/bin/env bash
# Derive autoware_sim_profile's copies of autoware_launch's control component and MPC parameters.
#
# The copies exist for one change (issue 025): MPC's automatic steering offset removal is
# off on CARLA. The estimator averages `reported steering - atan(yaw_rate * L / v)` over
# every control cycle above 5.56 m/s with a small steering angle -- collisions and
# transients included -- in a 1000-sample window that lives as long as the controller, and
# adds the result (clamped to 0.02 rad) to every steering command. CARLA's steering has no
# mechanical offset and acb reports the commanded angle, so all it learns is the history of
# the stack: a long-lived ego stack steered differently from a fresh one.
# upstream hard-codes the lateral parameter file inside tier4_control_component.launch.xml,
# so the component copy makes it an argument (default: upstream's file). The vehicle command
# gate's parameter file too (its velocity, acceleration and lateral-acceleration limits), for a
# car that drives past them (autoware-drift, swb roadmap 057: vel_lim 25 m/s, lateral 4-5 m/s^2).
# control_validator's file is the profile's own (a latency bound a stepped /clock can meet, e488ab0).
# And the longitudinal controller's (pid.param.yaml), whose slope compensation the profile lets
# reach steeper roads (swb roadmap 072, the NTU car-park ramp at 10.9 deg): max_pitch_rad and
# min_pitch_rad +/-0.2 instead of +/-0.1, so the pitch it compensates gravity for is no longer
# capped at 5.7 deg (0.98 m/s^2 of the 1.85 the ramp pulls with).
#
#   scripts/sync_control_component.sh          regenerate both copies from the installed upstream
#   scripts/sync_control_component.sh --check  exit 1 if either copy differs from what it would regenerate
#
# Run --check after an Autoware upgrade; regenerate if it fails.
set -euo pipefail
here="$(cd "$(dirname "$0")/.." && pwd)"
share="${AUTOWARE_LAUNCH_SHARE:-/opt/autoware/1.5.0/share/autoware_launch}"
up_component="$share/launch/components/tier4_control_component.launch.xml"
up_mpc="$share/config/control/trajectory_follower/lateral/mpc.param.yaml"
up_pid="$share/config/control/trajectory_follower/longitudinal/pid.param.yaml"
component="$here/autoware_sim_profile/launch/components/tier4_control_component.launch.xml"
mpc="$here/autoware_sim_profile/config/control/trajectory_follower/lateral/mpc.param.yaml"
pid="$here/autoware_sim_profile/config/control/trajectory_follower/longitudinal/pid.param.yaml"
lon_value='value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/$(var latlon_controller_param_path_dir)/longitudinal/$(var longitudinal_controller_mode).param.yaml"'
pitch_max='    max_pitch_rad: 0.1'
cv_value='<arg name="control_validator_param_path" value="$(find-pkg-share autoware_launch)/config/control/control_validator/control_validator.param.yaml"/>'
pitch_min='    min_pitch_rad: -0.1'
latlon_unless='<let name="latlon_controller_param_path_dir" value="" unless="$(var use_individual_control_param)"/>'
[ "$(grep -cF "$latlon_unless" "$up_component")" = 1 ] || {
    echo "upstream's component no longer has: $latlon_unless" >&2; exit 1; }
lat_value='value="$(find-pkg-share autoware_launch)/config/control/trajectory_follower/$(var latlon_controller_param_path_dir)/lateral/$(var lateral_controller_mode).param.yaml"'
offset_on='enable_auto_steering_offset_removal: true'
gate_value='<arg name="vehicle_cmd_gate_param_path" value="$(find-pkg-share autoware_launch)/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml"/>'
for f in "$up_component" "$up_mpc" "$up_pid"; do [ -f "$f" ] || { echo "no upstream file $f"; exit 2; }; done
[ "$(grep -cF "$lat_value" "$up_component")" = 1 ] || {
    echo "upstream no longer hard-codes the lateral parameter path exactly once; revisit this copy"; exit 2; }
[ "$(grep -cF "$gate_value" "$up_component")" = 1 ] || {
    echo "upstream no longer hard-codes the vehicle command gate's parameter path exactly once; revisit this copy"; exit 2; }
[ "$(grep -cF "$lon_value" "$up_component")" = 1 ] || {
    echo "upstream no longer hard-codes the longitudinal parameter path exactly once; revisit this copy"; exit 2; }
[ "$(grep -cF "$cv_value" "$up_component")" = 1 ] || {
    echo "upstream no longer hard-codes control_validator's parameter path exactly once; revisit this copy"; exit 2; }
for v in "$pitch_max" "$pitch_min"; do
    [ "$(grep -cxF "$v" "$up_pid")" = 1 ] || {
        echo "upstream pid.param.yaml no longer has '$v' exactly once; revisit this copy"; exit 2; }
done
[ "$(grep -cF "$offset_on" "$up_mpc")" = 1 ] || {
    echo "upstream mpc.param.yaml no longer has '$offset_on' exactly once; revisit this copy"; exit 2; }

gen_component() {
    sed \
        -e "s|$(printf '%s' "$lat_value" | sed 's/[][\.*^$|]/\\&/g')|value=\"\$(var lat_controller_param_path)\"|" \
        -e "s|$(printf '%s' "$gate_value" | sed 's/[][\.*^$|]/\\&/g')|<arg name=\"vehicle_cmd_gate_param_path\" value=\"\$(var gate_param_path)\"/>|" \
        -e "s|$(printf '%s' "$lon_value" | sed 's/[][\.*^$|]/\\&/g')|value=\"\$(var lon_param_path)\"|" \
        -e "s|^\( *\)$(printf '%s' "$latlon_unless" | sed 's/[][\.*^$|]/\\&/g')\$|&\n\1<!-- the profile's longitudinal file, unless the vehicle has its own (use_individual_control_param) -->\n\1<let name=\"lon_param_path\" $(printf '%s' "$lon_value" | sed 's/[&|]/\\&/g') if=\"\$(var use_individual_control_param)\"/>\n\1<let name=\"lon_param_path\" value=\"\$(var lon_controller_param_path)\" unless=\"\$(var use_individual_control_param)\"/>|" \
        -e "s|^\( *\)$(printf '%s' "$cv_value" | sed 's/[][\.*^$|]/\\&/g')|\1<!-- the profile's: a latency bound a stepped /clock can meet -->\n\1<arg name=\"control_validator_param_path\" value=\"\$(find-pkg-share autoware_sim_profile)/config/control/control_validator/control_validator.param.yaml\"/>|" \
        -e "0,/<launch>/s|<launch>|<launch>\n  <!-- GENERATED by acb scripts/sync_control_component.sh from autoware_launch's\n       tier4_control_component.launch.xml; do not edit by hand. Only changes:\n       the lateral and longitudinal controllers' and the vehicle command gate's parameter files are arguments instead of hard-coded. -->\n  <arg name=\"lat_controller_param_path\" default=\"\$(find-pkg-share autoware_launch)/config/control/trajectory_follower/lateral/mpc.param.yaml\" description=\"lateral controller parameter file\"/>\n  <arg name=\"lon_controller_param_path\" default=\"\$(find-pkg-share autoware_launch)/config/control/trajectory_follower/longitudinal/pid.param.yaml\" description=\"longitudinal controller parameter file\"/>\n  <arg name=\"gate_param_path\" default=\"\$(find-pkg-share autoware_launch)/config/control/vehicle_cmd_gate/vehicle_cmd_gate.param.yaml\" description=\"vehicle command gate parameter file\"/>|" \
        "$up_component"
}

gen_pid() {
    printf '%s\n' "# GENERATED by acb scripts/sync_control_component.sh from autoware_launch's" \
        "# config/control/trajectory_follower/longitudinal/pid.param.yaml; do not edit by hand." \
        "# Only change: max_pitch_rad / min_pitch_rad 0.2 / -0.2 (upstream 0.1 / -0.1; swb roadmap 072)."
    sed -e "s|^$pitch_max\$|    max_pitch_rad: 0.2|" -e "s|^$pitch_min\$|    min_pitch_rad: -0.2|" "$up_pid"
}

gen_mpc() {
    printf '%s\n' "# GENERATED by acb scripts/sync_control_component.sh from autoware_launch's" \
        "# config/control/trajectory_follower/lateral/mpc.param.yaml; do not edit by hand." \
        "# Only change: steering_offset.enable_auto_steering_offset_removal is false (issue 025)."
    sed -e "s|$offset_on|enable_auto_steering_offset_removal: false|" "$up_mpc"
}

if [ "${1:-}" = "--check" ]; then
    rc=0
    diff -q <(gen_component) "$component" >/dev/null || { echo "control component copy is out of date with $up_component"; rc=1; }
    diff -q <(gen_mpc) "$mpc" >/dev/null || { echo "MPC parameter copy is out of date with $up_mpc"; rc=1; }
    diff -q <(gen_pid) "$pid" >/dev/null || { echo "PID parameter copy is out of date with $up_pid"; rc=1; }
    [ $rc = 0 ] && echo "control component, MPC and PID copies match upstream" || echo "run scripts/sync_control_component.sh"
    exit $rc
else
    mkdir -p "$(dirname "$mpc")"
    gen_component > "$component"; echo "wrote $component"
    gen_mpc > "$mpc"; echo "wrote $mpc"
    mkdir -p "$(dirname "$pid")"
    gen_pid > "$pid"; echo "wrote $pid"
fi
