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


# autoware.launch.xml's per-release directories (autoware_release, roadmap 077), at 1.5.0: the
# checks below read the file as the 1.5.0 stack resolves it
RELEASE_15 = {
    "$(var sim_config_dir)": "$(find-pkg-share autoware_sim_profile)/config",
    "$(var up_control_config_dir)": "$(find-pkg-share autoware_launch)/config/control",
    "$(var up_planning_config_dir)": "$(find-pkg-share autoware_launch)/config/planning",
}


def read(rel):
    text = (PROFILE / rel).read_text()
    if rel == LAUNCH:
        for var, value in RELEASE_15.items():
            text = text.replace(var, value)
    return text


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
    """Issue 027, and scensim issue 190: the tracker reads the file
    multi_object_tracker_node_param_path names, whose default in the profile's launch and in the
    component is the profile's copy, byte for byte the path the component passed before the
    argument existed. A run that does not set it reads the same file."""
    profile_path = f"$(find-pkg-share autoware_sim_profile)/{TRACKER}"
    name = "multi_object_tracker_node_param_path"
    launch = read(LAUNCH)
    assert arg_default(launch, name) == profile_path
    assert f'<arg name="{name}" value="$(var {name})"/>' in launch
    component = read("launch/components/tier4_perception_component.launch.xml")
    assert arg_default(component, name) == profile_path
    assert ('name="object_recognition_tracking_multi_object_tracker_node_param_path"\n'
            f'      value="$(var {name})"') in component
    assert f"$(find-pkg-share autoware_launch)/{TRACKER}" not in component
    assert (PROFILE / TRACKER).is_file()


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


MVP = "config/planning/scenario_planning/lane_driving/motion_planning/motion_velocity_planner/"


@pytest.mark.parametrize("name,file", [("obstacle_cruise_param_path", "obstacle_cruise.param.yaml"),
                                       ("obstacle_stop_param_path", "obstacle_stop.param.yaml")])
def test_obstacle_param_files_default_to_upstreams_and_reach_the_planner(name, file):
    """Carla-scenario-bridge roadmap 074 E3: the cruise and stop parameter files are arguments
    (a knob setting is chosen per stack start), upstream's by default, passed through the
    planning component to the motion velocity planner's module."""
    default = f"$(find-pkg-share autoware_launch)/{MVP}{file}"
    assert arg_default(read(LAUNCH), name) == default
    assert f'<arg name="{name}" value="$(var {name})"/>' in read(LAUNCH)
    component = read("launch/components/tier4_planning_component.launch.xml")
    assert f'<arg name="{name}" default="{default}"' in component
    module = file.split(".")[0]
    assert f'<arg name="motion_velocity_planner_{module}_module_param_path" value="$(var {name})"/>' in component
    upstream(MVP + file)


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


def test_centerpoint_is_full_centerpoint_in_both_releases():
    """`lidar_detection_model:=centerpoint` runs full CenterPoint (roadmap 074 S5: contacts 1 of 8
    against centerpoint_tiny's 5), not upstream's default model, tiny; clustering stays the
    default and every type/name passes as given."""
    launch = read(LAUNCH)
    assert "<arg name=\"lidar_detection_model\" default=\"clustering\"" in launch
    resolve = ("'centerpoint/centerpoint' if '$(var lidar_detection_model)' == 'centerpoint' "
               "else '$(var lidar_detection_model)'")
    assert resolve in launch.replace("&quot;", "")
    assert "<arg name=\"lidar_detection_model\" value=\"$(var lidar_detector)\"/>" in launch
    v2 = read("launch/autoware-2.0.0.launch.xml")
    assert "<arg name=\"lidar_detection_model\" value=\"$(var lidar_detector)\"/>" in v2
    for text in (launch, v2):
        assert "value=\"$(var lidar_detection_model)\"" not in text


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
    launch = read(LAUNCH)
    for new in m.RENAMED.values():
        assert f'<arg name="{new}"' in launch, new


def test_ndt_defaults_to_autowares_and_cuda_swaps_the_localization_launch():
    """scensim roadmap 057: ndt:=cuda runs cuda_ndt_matcher's localization launch with this
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
    """scensim roadmap 063 d: ndt:=none is the profile's localization component with pose_source
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


def test_release_dirs_name_both_releases():
    """autoware_release (roadmap 077): every per-release directory is set for 1.5.0 and for 2.0.0,
    and 1.5.0's are what the profile named before the release argument."""
    launch = (PROFILE / LAUNCH).read_text()
    for var, value in RELEASE_15.items():
        name = var[len("$(var "):-1]
        assert f'<let name="{name}" value="{value}" if="$(var aw15)"/>' in launch
        assert f'<let name="{name}" value="' in launch.split('if="$(var aw15)"/>', 1)[1]
        assert launch.count(f'<let name="{name}" ') == 2


def test_release_2_0_0_has_every_ported_file():
    """Every file the 2.0.0 profile names under config/2.0.0 exists (scripts/port_config.py)."""
    import re
    text = (PROFILE / LAUNCH).read_text() + (PROFILE / "launch/autoware-2.0.0.launch.xml").read_text()
    root = PROFILE / "config/2.0.0"
    names = set(re.findall(r"\$\(var sim_config_dir\)/([A-Za-z0-9_./-]+\.ya?ml)", text))
    names |= {"localization/" + n for n in re.findall(r"\$\(var sim_loc_config_path\)/([A-Za-z0-9_./-]+\.ya?ml)", text)}
    assert names
    missing = sorted(n for n in names if not (root / n).is_file())
    assert not missing, missing


BVP = "config/planning/scenario_planning/lane_driving/behavior_planning/behavior_velocity_planner/"


def test_intersection_param_file_defaults_to_upstreams_and_reaches_the_planner():
    """Carla-scenario-bridge roadmap 074 N5: the intersection module's parameter file is an
    argument, upstream's by default on both releases, passed through the planning component (1.5.0)
    and to autoware_planning_launch (2.0.0); the profile's copy differs from upstream's only in
    occlusion.enable."""
    name = "intersection_param_path"
    launch = read(LAUNCH)
    assert arg_default(launch, name) == f"$(find-pkg-share autoware_launch)/{BVP}intersection.param.yaml"
    assert f'<arg name="{name}" value="$(var {name})"/>' in launch
    component = read("launch/components/tier4_planning_component.launch.xml")
    assert f'<arg name="{name}" default="$(find-pkg-share autoware_launch)/{BVP}intersection.param.yaml"' in component
    assert f'<arg name="behavior_velocity_planner_intersection_module_param_path" value="$(var {name})"/>' in component
    assert f'<arg name="behavior_velocity_planner_intersection_module_param_path" value="$(var {name})"/>' \
        in read("launch/autoware-2.0.0.launch.xml")
    up = [ln for ln in upstream(BVP + "intersection.param.yaml").read_text().splitlines()]
    ours = [ln for ln in read("config/planning/behavior_velocity_planner/intersection_occlusion.param.yaml").splitlines()
            if not ln.startswith("#")]
    diff = [(a, b) for a, b in zip(up, ours) if a != b]
    assert len(up) == len(ours) and diff == [("        enable: false", "        enable: true")]
    assert up[up.index("        enable: false") - 1].strip() == "occlusion:"


# --- Autoware 2.1.0: 2.0.0's profile plus universe 0.53.2's delta -----------------------------

UPSTREAM_21 = Path("/opt/autoware/2.1.0/share/autoware_perception_config")


def test_release_2_1_0_is_a_choice_with_2_0_0s_refusals():
    m = _check_launch_args()
    assert m.problems(["autoware_release:=2.1.0"]) == []
    assert m.problems(["autoware_release:=2.2.0"])
    p = m.problems(["autoware_release:=2.1.0", "ndt:=cuda"])
    assert len(p) == 1 and "on autoware_release:=2.1.0 use ndt:=autoware" in p[0]
    p = m.problems(["autoware_release:=2.1.0", "use_multi_channel_tracker_merger:=false"])
    assert len(p) == 1 and "Autoware 2.1.0's tracker" in p[0]


def test_release_2_1_0_runs_2_0_0s_profile():
    """2.1.0 takes every per-release directory and the 2.0.0 launch file that 2.0.0 does."""
    launch = (PROFILE / LAUNCH).read_text()
    assert "aw20" not in launch
    assert "'$(var autoware_release)' in ('2.0.0', '2.1.0')" in launch
    assert '<group if="$(var aw2x)" scoped="false">' in launch
    for name in ("sim_config_dir", "up_control_config_dir", "up_planning_config_dir"):
        assert launch.count(f'<let name="{name}" ') == 2
        assert f'<let name="{name}" value="' in launch.split('if="$(var aw15)"/>', 1)[1]


def test_release_2_1_0_takes_upstreams_prediction_parameters():
    """universe 0.53.2 declares lane_change_path_suppression.* with no default; 2.1.0's own
    autoware_perception_config has them, so the profile passes no copy (it had one until the
    install showed the keys, 2026-11-18)."""
    v2 = (PROFILE / "launch/autoware-2.0.0.launch.xml").read_text()
    assert "map_based_prediction_param_path" not in v2
    assert not (PROFILE / "config/2.1.0").exists()
    up = UPSTREAM_21 / "config/object_recognition/prediction/map_based_prediction.param.yaml"
    if not up.is_file():
        pytest.skip(f"no Autoware 2.1.0 at {up}")
    assert set(params(yaml.safe_load(up.read_text()))["lane_change_path_suppression"]) == {
        "enable", "keep_when_lane_ends", "lane_end_rescue_time_horizon"}


# --- the compare-map filter's loading, by the map (scensim issue 072) ---------------------------

FILTER = PROFILE / "scripts/pointcloud_map_filter_param"


def map_filter_param(*args):
    out = subprocess.run([sys.executable, str(FILTER), *map(str, args)],
                         capture_output=True, text=True)
    return out.returncode, out.stdout, out.stderr


def test_a_divided_map_gets_dynamic_loading_and_one_pcd_keeps_static(tmp_path):
    town = tmp_path / "Town01"   # every CARLA town: one PCD, and a metadata naming that file
    town.mkdir()
    (town / "pointcloud_map.pcd").write_bytes(b"")
    (town / "pointcloud_map_metadata.yaml").write_text(
        "x_resolution: 1000.0\ny_resolution: 1000.0\npointcloud_map.pcd: [-400, -400]\n")
    bare = tmp_path / "bare"     # one PCD, no metadata
    bare.mkdir()
    (bare / "pointcloud_map.pcd").write_bytes(b"")
    tiles_only = tmp_path / "tiles_only"   # tiles without their metadata: not a divided map
    (tiles_only / "pointcloud_map.pcd").mkdir(parents=True)
    (tiles_only / "pointcloud_map.pcd" / "a_0_0.pcd").write_bytes(b"")
    course = tmp_path / "haruna"  # divided: a directory of tiles and their metadata
    (course / "pointcloud_map.pcd").mkdir(parents=True)
    for name in ("haruna_0_0.pcd", "haruna_0_1.pcd"):
        (course / "pointcloud_map.pcd" / name).write_bytes(b"")
    (course / "pointcloud_map_metadata.yaml").write_text(
        "x_resolution: 100.0\ny_resolution: 100.0\nharuna_0_0.pcd: [0, 0]\nharuna_0_1.pcd: [0, 100]\n")
    for m in (town, bare, tiles_only, tmp_path / "missing"):
        assert map_filter_param(m, "/static.yaml", "/dynamic.yaml") == (0, "/static.yaml", "")
        code, out, err = map_filter_param("--mode", m)
        assert code == 0 and out.startswith("compare-map filter: static map loading") and not err
    assert map_filter_param(course, "/static.yaml", "/dynamic.yaml") == (0, "/dynamic.yaml", "")
    code, out, err = map_filter_param("--mode", course)
    assert code == 0 and not err
    assert out.startswith("compare-map filter: dynamic map loading, a divided map, 2 tiles")
    assert map_filter_param(course)[0] == 2


def test_the_launch_picks_the_compare_map_filter_by_the_map():
    """Both releases' perception take the compare-map filter's file from the script, and say
    which loading it chose; the component passes it through (sync_perception_component.sh)."""
    cmd = "$(find-pkg-prefix autoware_sim_profile)/lib/autoware_sim_profile/pointcloud_map_filter_param"
    rel = "perception/object_recognition/detection/pointcloud_filter/pointcloud_map_filter.param.yaml"
    launch = (PROFILE / LAUNCH).read_text()
    assert f"<log message=\"$(command '{cmd} --mode $(var map_path)')\"/>" in launch
    assert (f"value=\"$(command '{cmd} $(var map_path) $(find-pkg-share autoware_sim_profile)/config/"
            f"{rel} $(find-pkg-share autoware_launch)/config/{rel}')\"/>") in launch
    component = (PROFILE / "launch/components/tier4_perception_component.launch.xml").read_text()
    assert ("<arg name=\"pointcloud_map_filter_param_path\" default=\"$(find-pkg-share "
            f"autoware_sim_profile)/config/{rel}\"") in component
    assert 'value="$(var pointcloud_map_filter_param_path)"' in component
    v2 = (PROFILE / "launch/autoware-2.0.0.launch.xml").read_text()
    assert f"<log message=\"$(command '{cmd} --mode $(var map_path)')\"/>" in v2
    assert "$(find-pkg-share autoware_perception_config)/config/object_recognition/detection/" in v2
    # the profile's own files stay static (acb issue 029); upstream's are the dynamic ones
    static15 = yaml.safe_load((PROFILE / "config" / rel).read_text())
    assert static15["/**"]["ros__parameters"]["use_dynamic_map_loading"] is False
    up15 = UPSTREAM / "config" / rel
    if up15.is_file():
        assert yaml.safe_load(up15.read_text())["/**"]["ros__parameters"]["use_dynamic_map_loading"]
    up2 = Path("/opt/autoware/2.1.0/share/autoware_perception_config/config/object_recognition/"
               "detection/pointcloud_filter/voxel_based_compare_map_filter.param.yaml")
    if up2.is_file():
        assert yaml.safe_load(up2.read_text())["/**"]["ros__parameters"]["use_dynamic_map_loading"]
