"""Autoware's packages for every simulated car Autoware drives, from its vehicle spec (roadmap
051). A spec that says `autoware_car: false` (an RL policy's car, scensim 094 P2's MuSHR) gets
none and is reported skipped; an Autoware car without sensors is refused, naming both.

    python3 -m autoware_gen [--specs deps/sim_vehicle_spec/vehicles] [--root .]           # write
    python3 -m autoware_gen ... --check   # exit 1 when an output is stale or a car's pedal maps
                                          # were measured against other physics

For each spec `<name>` (its car):

    cars/<name>/<name>_description/            vehicle_info, mirror (from the spec), the pedal maps
                                               (measured, not written here), the RViz body
    cars/<name>/<name>_launch/                 the vehicle launch (no vehicle interface)
    cars/<name>/<name>_sensor_kit_description/ the calibration and sensor_kit.xacro (from the spec)
    cars/<name>/<name>_sensor_kit_launch/      the sensing pipeline
    individual_params/config/default/<name>_sensor_kit/   the calibration, as Autoware looks it up

Autoware selects them with vehicle_model:=<name> sensor_model:=<name>_sensor_kit.

A spec with bodies (sim_vehicle_spec's `bodies`: the body each simulator drives) is generated as
each of them (spec.effective). Its default body, the first, is `<name>` as above; each other
body `<b>` gets its own vehicle packages, `<name>_<b>_description` and `<name>_<b>_launch` (`<b>`
with `_` for every other character: `tesla_model3_carla_1_0`), holding that body's
vehicle_info, RViz body and pedal maps; the sensor kit is the car's, on every body. Autoware
reads `<vehicle_model>_description/config/vehicle_info.param.yaml` in several places (global
parameters, the vehicle URDF, the pointcloud preprocessor), so a body is a vehicle_model:
vehicle_model:=tesla_model3_carla_1_0 sensor_model:=tesla_model3_sensor_kit. The fixed files
are templates (autoware_gen/templates, @CAR@ and @KIT@ the package names); the rest is computed
from the spec with sim_vehicle_spec's library. Every computed file names the spec and its
spec_hash.
"""
import argparse
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT / "deps/sim_vehicle_spec"))

from sim_vehicle_spec import emit  # noqa: E402
from sim_vehicle_spec import spec as specmod  # noqa: E402

TOOL = "autoware_gen (autoware_sim_vehicle `just generate`)"
TEMPLATES = HERE / "templates"
PEDAL_MAPS = ("accel_map.csv", "brake_map.csv")


def names(spec):
    return spec.name, f"{spec.name}_sensor_kit"


class NotAnAutowareCar(ValueError):
    pass


def specs(specs_dir, skipped=None):
    """The specs Autoware drives, in name order. A spec with `autoware_car: false` is left out
    (its name appended to `skipped`); an Autoware car without sensors is refused: Autoware needs
    them, and a spec that lost them by mistake must not lose its packages in silence."""
    out = []
    for path in sorted(Path(specs_dir).glob("*.yaml")):
        source = specmod.load(path)
        if not specmod.autoware_car(source):
            if skipped is not None:
                skipped.append(f"{source.name} ({path.name}: autoware_car: false)")
            continue
        if not (source.source or source.doc).get("sensors"):
            raise NotAnAutowareCar(
                f"{path.name}: {source.name} is an Autoware car (no autoware_car: false) and has "
                "no sensors: give it its sensors, or say autoware_car: false")
        out.append((path, source))
    return out


def as_bodies(spec):
    """[(body, the spec as it)], the default first; [(None, spec)] without bodies."""
    return [(b, specmod.effective(spec, b)) for b in specmod.bodies(spec)] or [(None, spec)]


def vehicle_model(spec, body, default):
    """The vehicle_model Autoware runs `body` of `spec` as: the spec's name for its default
    body, `<name>_<body>` (non-alphanumerics as `_`) for another."""
    if body == default:
        return spec.name
    return f"{spec.name}_{re.sub('[^a-z0-9]+', '_', body.lower()).strip('_')}"


# --- the vehicle description ----------------------------------------------------------------

def vehicle_info(spec, header):
    g = spec.doc["geometry"]
    length = g["wheel_base"] + g["front_overhang"] + g["rear_overhang"]
    width = g["wheel_tread"] + g["left_overhang"] + g["right_overhang"]
    rows = [
        ("wheel_radius", "ground to wheel centre"),
        ("wheel_width", "lateral width of the tire"),
        ("wheel_base", "front to rear axle centre"),
        ("wheel_tread", "left to right wheel centre"),
        ("front_overhang", "front axle to the front edge"),
        ("rear_overhang", "rear axle to the rear edge"),
        ("left_overhang", "left wheels to the left edge"),
        ("right_overhang", "right wheels to the right edge"),
        ("vehicle_height", "ground to the roof"),
        ("max_steer_angle", "bicycle-model tire angle limit, rad"),
    ]
    out = emit.comment(header) + emit.comment(
        f"Autoware's vehicle_info for {spec.name} (autoware_vehicle_info_utils::VehicleInfo).\n"
        f"Length {emit.num(length)} m, width {emit.num(width)} m, height "
        f"{emit.num(g['vehicle_height'])} m.")
    out += "/**:\n  ros__parameters:\n"
    for key, what in rows:
        out += f"    {key}: {emit.fmt(g[key])}  # {what}\n"
    return out


def mirror(spec, header):
    m = spec.doc["mirror"]
    out = emit.comment(header) + "/**:\n  ros__parameters:\n"
    for key in specmod.SECTIONS["mirror"][0]:
        out += f"    {key}: {emit.fmt(m[key])}\n"
    return out


# --- the sensor kit -------------------------------------------------------------------------

def _mount(m, indent):
    pad = " " * indent
    return "".join(f"{pad}{k}: {emit.fmt(m[k])}\n" for k in specmod.MOUNT_KEYS)


def sensors_calibration(spec, header):
    out = emit.comment(header) + emit.comment("base_link -> sensor_kit_base_link (the kit's mount).")
    out += "base_link:\n  sensor_kit_base_link:\n" + _mount(spec.doc["sensor_kit"]["mount"], 4)
    return out


def sensor_kit_calibration(spec, header):
    out = emit.comment(header) + emit.comment(
        "Each sensor's mount in sensor_kit_base_link: metres (x forward, y left, z up), radians.")
    out += "sensor_kit_base_link:\n"
    for name, s in specmod.sensors(spec).items():
        out += f"  # {name}: {s['type']}\n  {s['key']}:\n" + _mount(s["mount"], 4)
    return out


OPTICAL_RPY = "-1.5707963267948966 0 -1.5707963267948966"


def _joint(link):
    return link.replace("/", "_") + "_joint"


def sensor_kit_xacro(spec, header):
    _, kit = names(spec)
    lines = [
        '<?xml version="1.0"?>',
        f"<!-- {header} -->",
        f'<robot name="{kit}" xmlns:xacro="http://ros.org/wiki/xacro">',
        "  <!-- The sensor kit's links, each placed from sensor_kit_calibration.yaml under",
        "       config_dir (Autoware's vehicle launch passes the description's config). -->",
        f'  <xacro:arg name="config_dir" default="$(find {kit}_description)/config"/>',
        "",
        '  <xacro:macro name="sensor_kit_macro" params="parent x y z roll pitch yaw">',
        "    <xacro:property name=\"calibration\" value=\"${xacro.load_yaml('$(arg config_dir)/sensor_kit_calibration.yaml')}\"/>",
        '    <joint name="sensor_kit_base_link_joint" type="fixed">',
        '      <origin rpy="${roll} ${pitch} ${yaw}" xyz="${x} ${y} ${z}"/>',
        '      <parent link="${parent}"/>',
        '      <child link="sensor_kit_base_link"/>',
        "    </joint>",
        '    <link name="sensor_kit_base_link"/>',
    ]
    for name, s in specmod.sensors(spec).items():
        key, link, frame = s["key"], s["link"], s["frame"]
        c = f"calibration['sensor_kit_base_link']['{key}']"
        lines += [
            "",
            f"    <!-- {name} ({s['type']}) published in {frame} on {s['topic']} -->",
            f'    <joint name="{_joint(link)}" type="fixed">',
            f'      <origin xyz="${{{c}[\'x\']}} ${{{c}[\'y\']}} ${{{c}[\'z\']}}"',
            f'              rpy="${{{c}[\'roll\']}} ${{{c}[\'pitch\']}} ${{{c}[\'yaw\']}}"/>',
            '      <parent link="sensor_kit_base_link"/>',
            f'      <child link="{link}"/>',
            "    </joint>",
            f'    <link name="{link}"/>',
        ]
        if frame != link:
            rpy = OPTICAL_RPY if s["type"] == "camera" else "0 0 0"
            lines += [
                f'    <joint name="{_joint(frame)}" type="fixed">',
                f'      <origin xyz="0 0 0" rpy="{rpy}"/>',
                f'      <parent link="{link}"/>',
                f'      <child link="{frame}"/>',
                "    </joint>",
                f'    <link name="{frame}"/>',
            ]
    lines += ["  </xacro:macro>", "</robot>", ""]
    return "\n".join(lines)


def spec_hash_file(spec, header):
    """What the vehicle agent reports when it registers with a scenario's relay (roadmap 051):
    the spec these packages were generated from, and the body they describe."""
    body = f"body: {spec.body}\n" if spec.body else ""
    return (emit.comment(header)
            + f"spec_hash: {specmod.spec_hash(spec)}\nname: {spec.name}\n{body}")


# --- the pedal maps -------------------------------------------------------------------------

def _carla_physics(spec):
    c = spec.section("carla")
    return c.get("physics") or c.get("physics_1_0")


def pedal_maps_problem(spec, root, car=None, default=True):
    """Why a car's pedal maps cannot be used with its spec (as one body: spec.effective), or
    None. They are measured on CARLA against the body's physics (roadmap 051 K3) and say so in
    pedal_maps.yaml. A body other than the default may carry another body's maps until its
    own are measured: `measured_on: provisional`, with `copied_from` and a `reason`, and no
    physics_hash (they were not measured on this physics)."""
    import yaml

    car = car or names(spec)[0]
    config = Path(root) / f"cars/{spec.name}/{car}_description/config"
    meta = config / "pedal_maps.yaml"
    if meta.is_file():
        recorded = yaml.safe_load(meta.read_text()) or {}
        if recorded.get("measured_on") == "provisional":
            if default:
                return f"{meta}: measured_on: provisional, but this is the car's default body"
            if "physics_hash" in recorded:
                return f"{meta}: measured_on: provisional claims no physics_hash"
            for key in ("copied_from", "reason"):
                if not str(recorded.get(key) or "").strip():
                    return f"{meta}: measured_on: provisional needs {key}"
            missing = [m for m in PEDAL_MAPS if not (config / m).is_file()]
            return f"{config}: no {', '.join(missing)}" if missing else None
        if recorded.get("measured_on") == "none":
            # A car with no CARLA blueprint to measure on, whose packages here no stack drives
            # (the golf cart runs its own Autoware and its own command converter). Allowed only
            # without CARLA physics, and only with the reason written down.
            if _carla_physics(spec):
                return f"{meta}: measured_on: none, but the spec gives CARLA physics to measure on"
            if not str(recorded.get("reason") or "").strip():
                return f"{meta}: measured_on: none needs a reason"
            return None
    missing = [m for m in PEDAL_MAPS if not (config / m).is_file()]
    if missing:
        return f"{config}: no {', '.join(missing)} (measure them: acb's build_longitudinal_maps.py)"
    if not meta.is_file():
        return f"{meta}: missing (the physics_hash the maps were measured against)"
    recorded = yaml.safe_load(meta.read_text()) or {}
    if not _carla_physics(spec):
        # CARLA runs the blueprint's own physics: the maps must have been measured on it.
        if recorded.get("measured_on") != "blueprint":
            return (f"{meta}: the spec gives CARLA no physics, so the maps must be measured on "
                    f"the blueprint's (measured_on: blueprint)")
        return None
    if recorded.get("physics_hash") != specmod.physics_hash(spec):
        return (f"{meta}: measured against physics_hash {str(recorded.get('physics_hash'))[:12]}, "
                f"the spec's is {specmod.physics_hash(spec)[:12]}: re-measure the pedal maps")
    return None


# --- driver ---------------------------------------------------------------------------------

def outputs(specs_dir, root):
    """{path relative to root: content}."""
    out = {}
    for path, source in specs(specs_dir):
        builds = as_bodies(source)
        spec = builds[0][1]
        name, kit = names(spec)
        rel = f"sim_vehicle_spec/vehicles/{path.name}"
        header = emit.header(spec, TOOL, rel)
        base = f"cars/{name}"
        for t in sorted(TEMPLATES.rglob("*")):
            parts = t.relative_to(TEMPLATES).parts
            if (not t.is_file() or t.name in ("COLCON_IGNORE", "CATKIN_IGNORE")
                    or parts[0] == "individual_params"):
                continue
            # the vehicle packages once per body, the sensor kit's once
            for body, _ in (builds if "@CAR@" in parts[0] else builds[:1]):
                car = vehicle_model(source, body, builds[0][0])
                dest = t.relative_to(TEMPLATES).as_posix().replace("@CAR@", car).replace(
                    "@KIT@", kit)
                out[f"{base}/{dest}"] = (t.read_text().replace("@CAR@", car)
                                         .replace("@KIT@", kit).replace("@NAME@", car))
        for body, e in builds:
            vd = f"{base}/{vehicle_model(source, body, builds[0][0])}_description"
            out[f"{vd}/config/vehicle_info.param.yaml"] = vehicle_info(e, header)
            out[f"{vd}/config/mirror.param.yaml"] = mirror(e, header)
            out[f"{vd}/config/spec_hash.yaml"] = spec_hash_file(e, header)
        kd = f"{base}/{kit}_description"
        sc, skc = sensors_calibration(spec, header), sensor_kit_calibration(spec, header)
        out[f"{kd}/config/sensors_calibration.yaml"] = sc
        out[f"{kd}/config/sensor_kit_calibration.yaml"] = skc
        out[f"{kd}/urdf/sensor_kit.xacro"] = sensor_kit_xacro(spec, header)
        out[f"individual_params/config/default/{kit}/sensors_calibration.yaml"] = sc
        out[f"individual_params/config/default/{kit}/sensor_kit_calibration.yaml"] = skc
    for t in sorted((TEMPLATES / "individual_params").rglob("*")):
        if t.is_file():
            out[t.relative_to(TEMPLATES).as_posix()] = t.read_text()
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--specs", type=Path, default=ROOT / "deps/sim_vehicle_spec/vehicles")
    ap.add_argument("--root", type=Path, default=ROOT)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    skipped = []
    try:
        specs(args.specs, skipped)
        out = outputs(args.specs, args.root)
    except NotAnAutowareCar as e:
        print(f"refused: {e}", file=sys.stderr)
        return 1
    for s in skipped:
        print(f"skipped: {s}, not an Autoware car", file=sys.stderr)
    problems = [f"stale: {p} (run `just generate`)" for p in emit.stale(out, args.root)]
    for path, source in specs(args.specs):
        builds = as_bodies(source)
        for body, e in builds:
            problem = pedal_maps_problem(e, args.root, vehicle_model(source, body, builds[0][0]),
                                         default=body == builds[0][0])
            if problem:
                problems.append(problem)
    if args.check:
        for p in problems:
            print(p, file=sys.stderr)
        return 1 if problems else 0
    for p in emit.write(out, args.root):
        print(f"wrote {p}")
    for p in problems:
        if not p.startswith("stale:"):
            print(f"note: {p}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
