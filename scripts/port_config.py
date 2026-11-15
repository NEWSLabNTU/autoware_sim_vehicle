#!/usr/bin/env python3
"""Port autoware_sim_profile's parameter files from Autoware 1.5.0 to another release.

The profile's files under autoware_sim_profile/config/ are upstream 1.5.0 files with the profile's
changes, or files of its own derived from one (the lane variants, the ndt:=none graph). For a
release R this writes autoware_sim_profile/config/R/<component>/...: the profile's changes from
1.5.0's upstream file, applied by key to R's upstream file, so R's own new keys, comments and
values stay R's.

- a parameter file: every leaf the profile changed is set in R's text in place (its line keeps
  R's comment and gains "profile: <1.5.0 value> -> <ours>");
- component_state_monitor's topics: entries matched by node_name_suffix;
- a diagnostic graph: units matched by path (changed, removed or added as the profile did), and
  its files list as the profile's;
- a variant is its primary's port with the variant's own changes applied the same way; the
  profile's own files in a release-independent form (COPIED) are copied;
- the profile's own paths, $(find-pkg-share autoware_sim_profile)/config/..., gain the release.

It stops on what it cannot carry over: a leaf R no longer has, or one R changed from 1.5.0 as
well (the profile's change may no longer fit). Each is listed in DECIDED below with what was
done, after a look at R's file; --check fails on any that is not.

  scripts/port_config.py --release 2.0.0 --upstream <share root of R> [--isolated] [--check]

<share root> holds each package's share tree (<root>/<pkg>/config/...), as
/opt/autoware/R/share does for a merged install; --isolated for <root>/<pkg>/share/<pkg> (the
2.0.0 image's /opt/autoware).
"""

import argparse
import copy
import filecmp
import os
import re
import sys
import tempfile

import yaml

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG = os.path.join(HERE, "autoware_sim_profile", "config")
UP15 = "/opt/autoware/1.5.0/share"
L15 = "autoware_launch"

# ours (under config/) -> (1.5.0 upstream: package, path), (R upstream: package, path), kind
FILES = {
    "control/control_validator/control_validator.param.yaml":
        ((L15, "config/control/control_validator/control_validator.param.yaml"),
         ("autoware_control_config", "config/control_validator/control_validator.param.yaml"), "param"),
    "control/trajectory_follower/lateral/mpc.param.yaml":
        ((L15, "config/control/trajectory_follower/lateral/mpc.param.yaml"),
         ("autoware_control_config", "config/trajectory_follower/lateral/mpc.param.yaml"), "param"),
    "control/trajectory_follower/longitudinal/pid.param.yaml":
        ((L15, "config/control/trajectory_follower/longitudinal/pid.param.yaml"),
         ("autoware_control_config", "config/trajectory_follower/longitudinal/pid.param.yaml"), "param"),
    "planning/behavior_path_planner/drivable_area_expansion.param.yaml":
        ((L15, "config/planning/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml"),
         ("autoware_planning_config", "config/scenario_planning/lane_driving/behavior_planning/behavior_path_planner/drivable_area_expansion.param.yaml"),
         "param"),
    "system/component_state_monitor/topics.yaml":
        ((L15, "config/system/component_state_monitor/topics.yaml"),
         ("autoware_launch", "config/system/component_state_monitor/topics.yaml"), "topics"),
    "system/diagnostics/system.yaml":
        ((L15, "config/system/diagnostics/system.yaml"), ("autoware_launch", "config/system/diagnostics/system.yaml"), "graph"),
    "system/diagnostics/autoware-sim.yaml":
        ((L15, "config/system/diagnostics/autoware-carla.yaml"), ("autoware_launch", "config/system/diagnostics/autoware-carla.yaml"),
         "graph"),
    "system/mrm_handler/mrm_handler.param.yaml":
        ((L15, "config/system/mrm_handler/mrm_handler.param.yaml"), ("autoware_launch", "config/system/mrm_handler/mrm_handler.param.yaml"),
         "param"),
}
for _name in ("ekf_localizer", "localization_error_monitor", "pose_initializer", "pose_instability_detector", "stop_filter",
              "twist2accel", "ndt_scan_matcher/ndt_scan_matcher",
              "ndt_scan_matcher/pointcloud_preprocessor/crop_box_filter_measurement_range",
              "ndt_scan_matcher/pointcloud_preprocessor/random_downsample_filter",
              "ndt_scan_matcher/pointcloud_preprocessor/voxel_grid_filter"):
    FILES[f"localization/{_name}.param.yaml"] = ((L15, f"config/localization/{_name}.param.yaml"),
                                                 ("autoware_localization_config", f"config/{_name}.param.yaml"), "param")
# 1.5.0's compare-map filter parameters are the voxel-based filter's file in 2.0.0
FILES["perception/object_recognition/detection/pointcloud_filter/pointcloud_map_filter.param.yaml"] = (
    (L15, "config/perception/object_recognition/detection/pointcloud_filter/pointcloud_map_filter.param.yaml"),
    ("autoware_perception_config", "config/object_recognition/detection/pointcloud_filter/voxel_based_compare_map_filter.param.yaml"),
    "param")

# the port's name where the release renamed the file
RENAMED = {"perception/object_recognition/detection/pointcloud_filter/pointcloud_map_filter.param.yaml":
           "perception/object_recognition/detection/pointcloud_filter/voxel_based_compare_map_filter.param.yaml"}
# variant -> its primary
VARIANTS = {
    "localization/ekf_localizer_lane.param.yaml": "localization/ekf_localizer.param.yaml",
    "localization/localization_error_monitor_lane.param.yaml": "localization/localization_error_monitor.param.yaml",
}
# the profile's own, in a form every release reads: copied (the ndt:=none graph is an edit of
# $(dirname)/autoware-sim.yaml, the port beside it)
COPIED = ("planning/preset/sim_preset.yaml", "system/diagnostics/autoware-sim-ndt-none.yaml")
# not ported: the tracker's UNKNOWN pruning GIoU is upstream's own 0.1 since autoware_launch #1885
NOT_PORTED = ("perception/object_recognition/tracking/multi_object_tracker/multi_object_tracker_node.param.yaml",)

# What could not be carried over by key, looked at by hand: (release, port, leaf or unit) -> what was done.
# "skip" leaves R's file as it is there; ("at", "<a.b.c>") applies the profile's value at R's new key.
DECIDED = {
    # 2.0.0 moved the latency bound into its own group
    ("2.0.0", "control/control_validator/control_validator.param.yaml", "thresholds.nominal_latency"):
        ("at", "latency_validator.nominal_latency"),
}


def upstream(root, pkg, rel, isolated):
    return os.path.join(root, pkg, "share", pkg, rel) if isolated else os.path.join(root, pkg, rel)


def load(path):
    with open(path) as f:
        return yaml.safe_load(f)


def leaves(d, p=()):
    if isinstance(d, dict):
        for k, v in d.items():
            yield from leaves(v, p + (k,))
    else:
        yield p, d


KEY = re.compile(r"^(\s*)([^\s#:][^:#]*?):(\s*)(.*?)(\s*#.*)?$")


def raw_values(text):
    """leaf path -> (line index, raw value text) for every 'key: scalar' line, by indentation."""
    stack, out = [], {}
    for i, line in enumerate(text.splitlines()):
        m = KEY.match(line)
        if not m or line.lstrip().startswith("- "):
            continue
        ind, key, val = len(m.group(1)), m.group(2).strip().strip("\"'"), m.group(4)
        while stack and stack[-1][0] >= ind:
            stack.pop()
        path = tuple(k for _, k in stack) + (key,)
        if val == "":
            stack.append((ind, key))
        else:
            out[path] = (i, val)
    return out


def short(path):
    return ".".join(str(k) for k in path if k not in ("/**", "ros__parameters"))


def port_param(ours, base, theirs, release, name, problems):
    o, b, t = load(ours), load(base), load(theirs)
    lo, lb, lt = dict(leaves(o)), dict(leaves(b)), dict(leaves(t))
    text = open(theirs).read()
    lines = text.splitlines()
    rt, ro = raw_values(text), raw_values(open(ours).read())
    for path in sorted(set(lo) | set(lb), key=str):
        if lo.get(path, "<absent>") == lb.get(path, "<absent>"):
            continue
        key = (release, name, short(path))
        decided = DECIDED.get(key)
        if decided == "skip":
            continue
        moved = isinstance(decided, tuple) and decided[0] == "at"
        value_from = path
        if moved:
            prefix = tuple(k for k in path if k in ("/**", "ros__parameters"))
            path = prefix + tuple(decided[1].split("."))
            decided = None
        if path not in lt or path not in rt:
            if decided is None:
                problems.append(f"{name}: {short(path)} ({lb.get(path)} -> {lo.get(path)}) is not in {release}'s file")
            continue
        if not moved and lt[path] != lb.get(path) and decided is None:
            problems.append(f"{name}: {short(path)}: {release} moved it too ({lb.get(path)} -> {lt[path]}); ours {lo.get(path)}")
            continue
        if value_from not in ro:
            problems.append(f"{name}: {short(value_from)} not found in the profile's text")
            continue
        i, _ = rt[path]
        new = ro[value_from][1] if decided is None else str(decided)
        m = KEY.match(lines[i])
        assert m
        comment = (m.group(5) or "").strip()
        was = lt[path] if moved else lb.get(path)
        note = f"profile: {yaml.safe_dump(was, default_flow_style=True).strip().removesuffix('...').strip()} -> {new}"
        lines[i] = f"{m.group(1)}{m.group(2)}:{m.group(3) or ' '}{new} # {note}" + (f"; {comment.lstrip('#').strip()}" if comment else "")
    return "\n".join(lines) + "\n"


def port_topics(ours, base, theirs, release, name, problems):
    o, b, t = load(ours), load(base), load(theirs)
    key = lambda e: e["args"]["node_name_suffix"]  # noqa: E731
    bo, bb = {key(e): e for e in o}, {key(e): e for e in b}
    out = []
    for e in t:
        k = key(e)
        if k in bo and k in bb and bo[k] != bb[k]:
            if e != bb[k]:
                problems.append(f"{name}: {k}: {release} changed this entry too")
                out.append(e)
                continue
            out.append(copy.deepcopy(bo[k]))
        else:
            out.append(e)
    for k in bo:
        if k not in bb:
            out.append(bo[k])
    return "# The profile's component_state_monitor topics (" + release + "): upstream's, with the profile's\n" \
           "# simulation rates and timeouts per entry (scripts/port_config.py).\n" + yaml.safe_dump(out, sort_keys=False)


def port_graph(ours, base, theirs, release, name, problems):
    o, b, t = load(ours), load(base), load(theirs)
    uo = {u["path"]: u for u in o.get("units", [])}
    ub = {u["path"]: u for u in b.get("units", [])}
    out = copy.deepcopy(t)
    units = []
    for u in t.get("units", []):
        p = u["path"]
        if p in ub and p not in uo:
            continue  # the profile removed it
        if p in ub and uo[p] != ub[p]:
            if u != ub[p]:
                problems.append(f"{name}: unit {p}: {release} changed it too")
                units.append(u)
                continue
            units.append(copy.deepcopy(uo[p]))
        else:
            units.append(u)
    for p, u in uo.items():
        if p not in ub and p not in {x["path"] for x in units}:
            units.append(u)
    out["units"] = units
    if o.get("files") != b.get("files"):
        out["files"] = o["files"]
    head = "".join(line for line in open(ours) if line.startswith("#"))
    return head + yaml.safe_dump(out, sort_keys=False)


PORTERS = {"param": port_param, "topics": port_topics, "graph": port_graph}


def with_release(text, release):
    return text.replace("$(find-pkg-share autoware_sim_profile)/config/", f"$(find-pkg-share autoware_sim_profile)/config/{release}/")


def write(dest, rel, text):
    path = os.path.join(dest, rel)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        f.write(text)


def port(release, root, isolated, dest):
    problems = []
    kinds = {}
    for ours, ((p15, r15), (p20, r20), kind) in sorted(FILES.items()):
        base, theirs = os.path.join(UP15, p15, r15), upstream(root, p20, r20, isolated)
        for f in (base, theirs):
            if not os.path.isfile(f):
                problems.append(f"{ours}: no file {f}")
        if not (os.path.isfile(base) and os.path.isfile(theirs)):
            continue
        name = RENAMED.get(ours, ours)
        kinds[ours] = kind
        write(dest, name, with_release(PORTERS[kind](os.path.join(CONFIG, ours), base, theirs, release, name, problems), release))
    for var, prim in sorted(VARIANTS.items()):
        kind = kinds.get(prim)
        if kind is None:
            continue
        # the variant's change from its primary, onto the primary's port
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
            tmp.write(open(os.path.join(dest, prim)).read().replace(f"/config/{release}/", "/config/"))
        try:
            write(dest, var, with_release(PORTERS[kind](os.path.join(CONFIG, var), os.path.join(CONFIG, prim), tmp.name,
                                                        release, var, problems), release))
        finally:
            os.unlink(tmp.name)
    for rel in COPIED:
        write(dest, rel, with_release(open(os.path.join(CONFIG, rel)).read(), release))
    return problems


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--release", required=True)
    p.add_argument("--upstream", required=True, help="the release's share root")
    p.add_argument("--isolated", action="store_true", help="an isolated install: <root>/<pkg>/share/<pkg>")
    p.add_argument("--check", action="store_true")
    a = p.parse_args()
    dest = os.path.join(CONFIG, a.release)
    if a.check:
        with tempfile.TemporaryDirectory() as tmp:
            problems = port(a.release, a.upstream, a.isolated, tmp)
            for d, _, files in os.walk(tmp):
                for f in files:
                    rel = os.path.relpath(os.path.join(d, f), tmp)
                    mine = os.path.join(dest, rel)
                    if not os.path.isfile(mine) or not filecmp.cmp(mine, os.path.join(d, f), shallow=False):
                        problems.append(f"{rel}: the committed port differs from a fresh one")
        for m in problems:
            print(m)
        if problems:
            print(f"run scripts/port_config.py --release {a.release} --upstream {a.upstream}{' --isolated' if a.isolated else ''}")
            return 1
        print(f"the {a.release} port matches its upstream")
        return 0
    problems = port(a.release, a.upstream, a.isolated, dest)
    for m in problems:
        print(f"TO DECIDE: {m}")
    print(f"wrote {dest}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
