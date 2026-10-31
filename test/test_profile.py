"""The Autoware simulation profile (autoware_sim_profile): settings that are not code but that a
run depends on, checked here so a sync with upstream or an edit cannot drop them silently.
Moved from acb_bridge's tests/launch_profile.rs with the profile (roadmap 051)."""

import os
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest
import yaml

PROFILE = Path(__file__).resolve().parent.parent / "autoware_sim_profile"
UPSTREAM = Path("/opt/autoware/1.5.0/share/autoware_launch")
LAUNCH = "launch/autoware.launch.xml"
MPC = "config/control/trajectory_follower/lateral/mpc.param.yaml"
TRACKER = ("config/perception/object_recognition/tracking/multi_object_tracker/"
           "multi_object_tracker_node.param.yaml")
COMMON = "config/planning/scenario_planning/common/common.param.yaml"


def read(rel):
    return (PROFILE / rel).read_text()


def params(doc):
    return doc["/**"]["ros__parameters"]


def upstream(rel):
    path = UPSTREAM / rel
    if not path.exists():
        pytest.skip(f"no upstream at {path}")
    return path


def test_mpc_steering_offset_removal_is_off():
    """Issue 025: MPC's steering offset estimator learns from collisions and transients and
    never forgets across episodes; a simulator's steering has no offset to learn."""
    offset = params(yaml.safe_load(read(MPC)))["steering_offset"]
    assert offset["enable_auto_steering_offset_removal"] is False


def test_mpc_copy_matches_upstream_but_for_the_offset():
    up, ours = yaml.safe_load(upstream(MPC).read_text()), yaml.safe_load(read(MPC))
    for doc in (up, ours):
        params(doc)["steering_offset"]["enable_auto_steering_offset_removal"] = None
    assert up == ours, f"{MPC} drifted from upstream; run scripts/sync_control_component.sh"


def test_profile_launches_control_with_its_mpc():
    launch = read(LAUNCH)
    assert ("<arg name=\"launch_control\" value=\"$(eval &quot;'$(var control)' == 'true' and "
            "'$(var sim_control)' != 'true'&quot;)\"/>") in launch
    assert "$(find-pkg-share autoware_sim_profile)/launch/components/tier4_control_component.launch.xml" in launch
    assert f"$(find-pkg-share autoware_sim_profile)/{MPC}" in launch
    component = read("launch/components/tier4_control_component.launch.xml")
    assert "<arg name=\"lat_controller_param_path\" default=" in component
    assert "value=\"$(var lat_controller_param_path)\"" in component


def test_perception_off_keeps_everything_but_object_recognition():
    """Issue 026: `perception:=false` keeps obstacle segmentation, the occupancy grid and
    traffic light fusion, and drops only object recognition."""
    launch = read(LAUNCH)
    assert "<group if=\"$(eval &quot;'$(var sim_perception)' == 'true'&quot;)\">" in launch
    assert ("<arg name=\"object_recognition\" value=\"$(eval &quot;'$(var perception)' == 'true' "
            "and '$(var publish_ground_truth_objects)' != 'true'&quot;)\"/>") in launch
    component = read("launch/components/tier4_perception_component.launch.xml")
    assert "$(find-pkg-share autoware_sim_profile)/launch/components/perception.launch.xml" in component
    assert "<arg name=\"object_recognition\" value=\"$(var object_recognition)\"/>" in component
    perception = read("launch/components/perception.launch.xml")
    assert "<arg name=\"object_recognition\" default=\"true\"" in perception
    assert perception.count("'$(var object_recognition)'.lower() == 'true' and "
                            "'$(var use_empty_dynamic_object_publisher)'.lower()") == 2
    assert "<group unless=\"$(var use_empty_dynamic_object_publisher)\">" not in perception


def test_tracker_merges_only_overlapping_unknowns():
    """Issue 027: UNKNOWN's generalized-IoU pruning threshold is 0.1, not 1.5.0's -0.3."""
    giou = params(yaml.safe_load(read(TRACKER)))["pruning_generalized_iou_thresholds"]
    assert giou[0] == 0.1


def test_tracker_copy_matches_upstream_but_for_unknown_giou():
    up, ours = yaml.safe_load(upstream(TRACKER).read_text()), yaml.safe_load(read(TRACKER))
    for doc in (up, ours):
        params(doc)["pruning_generalized_iou_thresholds"][0] = None
    assert up == ours, f"{TRACKER} drifted from upstream; run scripts/sync_perception_component.sh"


def test_perception_component_passes_the_profile_tracker_params():
    component = read("launch/components/tier4_perception_component.launch.xml")
    assert f"value=\"$(find-pkg-share autoware_sim_profile)/{TRACKER}\"" in component
    assert f"$(find-pkg-share autoware_launch)/{TRACKER}" not in component


def arg_default(launch, name):
    at = launch.index(f"<arg name=\"{name}\" default=\"")
    rest = launch[at:]
    start = rest.index("default=\"") + len("default=\"")
    return rest[start:rest.index('"', start)]


def test_max_velocity_defaults_to_upstream_max_vel():
    """Roadmap 033: at its default the stack plans exactly as upstream (15 km/h)."""
    default = float(arg_default(read(LAUNCH), "max_velocity"))
    assert default == 4.17
    up = yaml.safe_load(upstream(COMMON).read_text())
    assert params(up)["max_vel"] == default


def test_profile_launches_planning_with_max_velocity():
    launch = read(LAUNCH)
    assert ("<arg name=\"launch_planning\" value=\"$(eval &quot;'$(var planning)' == 'true' and "
            "'$(var sim_planning)' != 'true'&quot;)\"/>") in launch
    assert ("<group if=\"$(eval &quot;'$(var planning)' == 'true' and '$(var sim_planning)' == "
            "'true'&quot;)\">") in launch
    assert "$(find-pkg-share autoware_sim_profile)/launch/components/tier4_planning_component.launch.xml" in launch
    assert ("value=\"$(command '$(find-pkg-prefix autoware_sim_profile)/lib/autoware_sim_profile/"
            "planning_common_param $(var planning_common_param_base) $(var max_velocity)')\"") in launch
    # the base is upstream's file unless a car names its own (autoware-drift)
    assert ("<arg name=\"planning_common_param_base\" default=\"$(find-pkg-share autoware_launch)/"
            "config/planning/scenario_planning/common/common.param.yaml\"") in launch
    assert "<node pkg=\"topic_tools\" exec=\"relay\" name=\"trajectory_relay\">" in launch
    component = read("launch/components/tier4_planning_component.launch.xml")
    assert "<arg name=\"planning_common_param_path\" default=" in component
    assert "<arg name=\"common_param_path\" value=\"$(var planning_common_param_path)\"/>" in component


def planning_common_param(upstream_file, max_velocity, cache):
    out = subprocess.run([sys.executable, str(PROFILE / "scripts/planning_common_param"),
                          str(upstream_file), max_velocity],
                         env={**os.environ, "XDG_CACHE_HOME": str(cache)},
                         capture_output=True, text=True)
    return out.returncode, out.stdout, out.stderr


def test_planning_common_param_changes_max_vel_only():
    """At upstream's value: upstream's path; at another: upstream's file with max_vel changed
    and nothing else; a bad value fails. Stdout is the bare path, stderr empty."""
    common = upstream(COMMON)
    with tempfile.TemporaryDirectory() as tmp:
        cache = Path(tmp) / "cache"
        assert planning_common_param(common, "4.17", cache) == (0, str(common), "")
        assert not cache.exists(), "the default must not write a file"
        for arg, want in (("11.11", 11.11), ("20", 20.0)):
            code, out, err = planning_common_param(common, arg, cache)
            assert (code, err) == (0, "")
            assert not out.endswith("\n") and out.startswith(str(cache))
            ours, up = yaml.safe_load(Path(out).read_text()), yaml.safe_load(common.read_text())
            assert params(ours)["max_vel"] == want and isinstance(params(ours)["max_vel"], float)
            params(ours)["max_vel"] = params(up)["max_vel"] = None
            assert up == ours, f"only max_vel may differ from upstream ({arg})"
        for bad in ("abc", "0", "-1", "nan", "inf"):
            code, out, _ = planning_common_param(common, bad, cache)
            assert (code, out) == (2, ""), bad


@pytest.mark.parametrize("rel", [LAUNCH,
                                 "launch/components/tier4_control_component.launch.xml",
                                 "launch/components/tier4_perception_component.launch.xml",
                                 "launch/components/tier4_planning_component.launch.xml",
                                 "launch/components/perception.launch.xml",
                                 "launch/components/tier4_localization_component.launch.xml"])
def test_launch_comments_have_no_double_hyphen(rel):
    """XML forbids `--` in a comment, and play_launch refuses the whole launch for it."""
    for comment in read(rel).split("<!--")[1:]:
        body = comment.split("-->")[0]
        assert "--" not in body, f"{rel}: comment contains '--': {body}"


def test_tracker_merger_choice_reaches_perception_with_upstream_default():
    """Issue 028: the choice reaches the perception component; Autoware 1.5's default (true)."""
    launch = read(LAUNCH)
    assert "<arg name=\"use_multi_channel_tracker_merger\" default=\"true\"" in launch
    assert ("<arg name=\"use_multi_channel_tracker_merger\" "
            "value=\"$(var use_multi_channel_tracker_merger)\"/>") in launch
    component = read("launch/components/tier4_perception_component.launch.xml")
    assert "<arg name=\"use_multi_channel_tracker_merger\" default=" in component
    assert ("<arg name=\"use_multi_channel_tracker_merger\" "
            "value=\"$(var use_multi_channel_tracker_merger)\"/>") in component


def test_the_profile_starts_no_vehicle_interface_and_no_agent():
    """Roadmap 051: the vehicle bridge starts its interface; the agent comes by extra_launch."""
    launch = read(LAUNCH)
    code = "".join(part.split("-->", 1)[-1] for part in launch.split("<!--"))
    for word in ("acb_bridge", "acb_pilot", "aib_", "exec=\"agent\"", "<arg name=\"relay\""):
        assert word not in code, word
    assert "<arg name=\"launch_vehicle_interface\" value=\"false\"/>" in code


def clock_guard():
    import importlib.machinery
    import importlib.util

    loader = importlib.machinery.SourceFileLoader("clock_guard", str(PROFILE / "scripts/clock_guard"))
    spec = importlib.util.spec_from_loader("clock_guard", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_clock_guard_ends_on_a_step_back_only():
    """Roadmap 051: an episode is the stack's lifetime; time going back ends it."""
    g = clock_guard()
    assert g.check(None, 5, 0) is None
    assert g.check(1_000_000_000_000, 1_000_050_000_000, 0) is None
    assert g.check(1_000_050_000_000, 1_000_050_000_000, 0) is None  # the same frame again
    assert g.check(1_300_000_000_000, 1_000_050_000_000, 0) == 299_950_000_000
    assert g.check(1_000_100_000_000, 1_000_050_000_000, 100_000_000) is None


def test_the_profile_starts_the_clock_guard_required():
    launch = read(LAUNCH)
    assert ('<node pkg="autoware_sim_profile" exec="clock_guard" name="clock_guard" '
            'output="screen"\n        on_exit="shutdown" if="$(var sim_clock_guard)"/>') in launch
    assert '<arg name="sim_clock_guard" default="true"' in launch


def _check_launch_args():
    import importlib.machinery
    import importlib.util
    path = PROFILE / "scripts" / "check_launch_args"
    loader = importlib.machinery.SourceFileLoader("check_launch_args", str(path))
    spec = importlib.util.spec_from_loader("check_launch_args", loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


def test_retired_launch_arguments_are_refused_by_name():
    m = _check_launch_args()
    assert m.problems(["map_path:=/x", "sim_control:=false", "vehicle_model:=tesla_model3"]) == []
    p = m.problems(["carla_control:=false", "vehicle_model:=acb_vehicle"])
    assert len(p) == 2
    assert "pass sim_control:=false" in p[0]
    assert "pass vehicle_model:=tesla_model3" in p[1]
    assert m.main(["carla_planning:=true"]) == 2


def test_every_renamed_argument_exists_in_the_profile():
    m = _check_launch_args()
    launch = (PROFILE / "launch" / "autoware.launch.xml").read_text()
    for new in m.RENAMED.values():
        assert f'<arg name="{new}"' in launch, new


def test_ndt_defaults_to_autowares_and_cuda_swaps_the_localization_launch():
    """swb roadmap 057: ndt:=cuda runs cuda_ndt_matcher's localization launch with this
    profile's localization parameters; the default stays Autoware's own component."""
    launch = read(LAUNCH)
    assert arg_default(launch, "ndt") == "autoware"
    assert "'$(var sim_localization)' == 'true' and '$(var ndt)' in ('autoware', 'none')" in launch
    assert "'$(var sim_localization)' == 'true' and '$(var ndt)' == 'cuda'" in launch
    assert "$(find-pkg-share cuda_ndt_matcher_launch)/launch/cuda_localization.launch.xml" in launch
    assert '<arg name="localization_config_path" value="$(var sim_loc_config_path)"/>' in launch


def test_ndt_takes_only_its_choices():
    m = _check_launch_args()
    for ok in ("autoware", "cuda", "none"):
        assert m.problems([f"ndt:={ok}"]) == [], ok
    assert m.problems(["ndt:=gpu"]) == ["ndt:=gpu is not one of autoware, cuda, none"]


LOC = "config/localization"
COMPONENT = "launch/components/tier4_localization_component.launch.xml"


def test_ndt_none_runs_the_component_without_a_scan_matcher():
    """swb roadmap 063 d: ndt:=none is the profile's localization component with pose_source
    none (no estimator upstream knows: no NDT, no NDT input filters, the pose initializer
    without alignment), and the EKF's pose input left on NDT's topic."""
    launch = read(LAUNCH)
    assert ("'$(var sim_localization)' == 'true' and '$(var ndt)' in ('autoware', 'none')"
            in launch)
    assert f"$(find-pkg-share autoware_sim_profile)/{COMPONENT}" in launch
    assert "$(find-pkg-share autoware_launch)/launch/components/tier4_localization_component" not in launch
    assert ("<arg name=\"pose_source\" value=\"$(eval &quot;'ndt' if '$(var ndt)' == "
            "'autoware' else 'none'&quot;)\"/>") in launch
    # tier4_localization_launch's estimator list: "none" must stay out of it
    estimator = Path("/opt/autoware/1.5.0/share/tier4_localization_launch/launch/"
                     "pose_twist_estimator/pose_twist_estimator.launch.xml")
    if estimator.exists():
        text = estimator.read_text()
        assert "<let name=\"available_args\" value=\"[\\'ndt\\',\\'yabloc\\',\\'eagleye\\',\\'artag\\',\\'lidar-marker\\']\"/>" in text
        assert '<arg name="ndt_enabled" value="$(var use_ndt_pose)"/>' in text
    fusion = Path("/opt/autoware/1.5.0/share/tier4_localization_launch/launch/"
                  "pose_twist_fusion_filter/pose_twist_fusion_filter.launch.xml")
    if fusion.exists():
        assert ('<arg name="input_pose_with_cov_name" '
                'value="/localization/pose_estimator/pose_with_covariance"/>') in fusion.read_text()


def test_localization_component_copy_is_generated():
    component = read(COMPONENT)
    assert "GENERATED by autoware_sim_vehicle scripts/sync_localization_component.sh" in component
    assert '<arg name="localization_error_monitor_param_path" value="$(var error_monitor_param_path)"/>' in component
    assert '<arg name="ekf_localizer_param_path" value="$(var ekf_param_path)"/>' in component
    if (UPSTREAM / "launch/components/tier4_localization_component.launch.xml").exists():
        out = subprocess.run([str(PROFILE.parent / "scripts/sync_localization_component.sh"), "--check"],
                             capture_output=True, text=True)
        assert out.returncode == 0, out.stdout + out.stderr


def test_ndt_none_judges_no_scan_matcher():
    """The default graph under ndt:=none is autoware-sim.yaml without NDT's status unit."""
    launch = read(LAUNCH)
    assert ("'$(find-pkg-share autoware_sim_profile)/config/system/diagnostics/"
            "autoware-sim-ndt-none.yaml' if '$(var ndt)' == 'none' else "
            "'$(find-pkg-share autoware_sim_profile)/config/system/diagnostics/autoware-sim.yaml'"
            in launch)
    graph = yaml.safe_load(read("config/system/diagnostics/autoware-sim-ndt-none.yaml"))
    assert graph["files"] == [{"path": "$(dirname)/autoware-sim.yaml"}]
    assert graph["edits"] == [{"type": "remove", "path": "/autoware/localization/scan_matching_status"}]
    assert "units" not in graph
    base = yaml.safe_load(read("config/system/diagnostics/autoware-sim.yaml"))
    assert "/autoware/localization/scan_matching_status" in [u["path"] for u in base["units"]]


def test_lane_thresholds_change_only_the_lateral_ellipse():
    """063 d: WARN and ERROR at 1.75 m across the heading (a WARN already ends autonomous
    mode); every other setting as the base file."""
    lateral = ("error_ellipse_size_lateral_direction", "warn_ellipse_size_lateral_direction")
    base = params(yaml.safe_load(read(f"{LOC}/localization_error_monitor.param.yaml")))
    lane = params(yaml.safe_load(read(f"{LOC}/localization_error_monitor_lane.param.yaml")))
    assert all(lane[k] == 1.75 for k in lateral) and lane["scale"] == 3.0
    for k in lateral:
        base[k] = lane[k] = None
    assert base == lane
    base = params(yaml.safe_load(read(f"{LOC}/ekf_localizer.param.yaml")))
    lane = params(yaml.safe_load(read(f"{LOC}/ekf_localizer_lane.param.yaml")))
    assert all(lane["diagnostics"][k] == 1.75 for k in lateral)
    for k in lateral:
        base["diagnostics"][k] = lane["diagnostics"][k] = None
    assert base == lane


def test_lane_files_are_selectable():
    launch = read(LAUNCH)
    assert arg_default(launch, "localization_error_monitor_param_path") == \
        "$(var sim_loc_config_path)/localization_error_monitor.param.yaml"
    assert arg_default(launch, "ekf_localizer_param_path") == \
        "$(var sim_loc_config_path)/ekf_localizer.param.yaml"
    assert '<arg name="error_monitor_param_path" value="$(var localization_error_monitor_param_path)"/>' in launch
    assert '<arg name="ekf_param_path" value="$(var ekf_localizer_param_path)"/>' in launch
    # cuda's launch takes the EKF's file, not the error monitor's
    assert '<arg name="ekf_localizer_param_path" value="$(var ekf_localizer_param_path)"/>' in launch


def test_pairs_that_would_launch_without_their_part_are_refused(tmp_path):
    m = _check_launch_args()
    monitor = "localization_error_monitor_param_path:=/x/lane.yaml"
    assert m.problems(["ndt:=none", monitor]) == []
    assert m.problems([monitor]) == []
    p = m.problems(["ndt:=cuda", monitor])
    assert len(p) == 1 and "does not reach ndt:=cuda" in p[0]
    ndt_graph = tmp_path / "g.yaml"
    ndt_graph.write_text("units:\n  - { path: /a, type: diag, node: ndt_scan_matcher, name: s }\n")
    p = m.problems(["ndt:=none", f"diagnostic_graph_path:={ndt_graph}"])
    assert len(p) == 1 and "judges ndt_scan_matcher" in p[0]
    assert m.problems(["ndt:=autoware", f"diagnostic_graph_path:={ndt_graph}"]) == []
    ours = PROFILE / "config/system/diagnostics/autoware-sim-ndt-none.yaml"
    assert m.problems(["ndt:=none", f"diagnostic_graph_path:={ours}"]) == []
