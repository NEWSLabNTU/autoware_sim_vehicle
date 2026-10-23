"""The contract: what a vehicle side publishes to Autoware, from the vehicle spec. Pure.

A topic passes when it has messages of the right type, at the rate it should (counted on
their own stamps, so a slow simulator is not a slow vehicle side), in the right frame, with
stamps that do not go back and stay near /clock. `/robot_description` must name every
sensor's link and frame.
"""
from dataclasses import dataclass, field

# The vehicle interface Autoware reads, whatever the car: (topic, type, minimum Hz, frame or
# None when the message has no header).
VEHICLE_INTERFACE = (
    ("/vehicle/status/velocity_status", "autoware_vehicle_msgs/msg/VelocityReport", 10.0,
     "base_link"),
    ("/vehicle/status/steering_status", "autoware_vehicle_msgs/msg/SteeringReport", 10.0, None),
    ("/vehicle/status/gear_status", "autoware_vehicle_msgs/msg/GearReport", 5.0, None),
    ("/vehicle/status/control_mode", "autoware_vehicle_msgs/msg/ControlModeReport", 5.0, None),
)
SENSOR_TYPES = {
    "lidar": "sensor_msgs/msg/PointCloud2",
    "camera": "sensor_msgs/msg/Image",
    "imu": "sensor_msgs/msg/Imu",
    "gnss": "sensor_msgs/msg/NavSatFix",
}
# A sensor with no rate in the spec must still reach this.
DEFAULT_MIN_HZ = 10.0
# What share of the spec's rate a sensor must reach, on its own stamps.
RATE_SHARE = 0.8
# How far a stamp may be from the latest /clock, s.
CLOCK_SLACK = 1.0


@dataclass
class Expected:
    topic: str
    type: str
    min_hz: float
    frame: str | None
    what: str


@dataclass
class Observed:
    """What arrived on a topic: the type(s) the graph has on it, and per message (stamp s,
    frame_id)."""
    type: str = ""
    samples: list = field(default_factory=list)
    # The /clock reading when each message arrived, s.
    clock_at_arrival: list = field(default_factory=list)


def sensors_for(spec, simulator):
    """The spec's sensors a vehicle side on `simulator` publishes: CARLA spawns those with a
    `carla` block, Isaac those enabled; `any` takes the sensors both have."""
    from sim_vehicle_spec import spec as specmod

    out = {}
    for name, s in specmod.sensors(spec).items():
        carla = bool(s.get("carla"))
        isaac = s["enabled"]
        if {"carla": carla, "isaac": isaac, "any": carla and isaac}[simulator]:
            out[name] = s
    return out


def expected(spec, simulator):
    out = [Expected(t, ty, hz, frame, "vehicle interface")
           for t, ty, hz, frame in VEHICLE_INTERFACE]
    for name, s in sensors_for(spec, simulator).items():
        hz = s.get("rate_hz")
        out.append(Expected(s["topic"], SENSOR_TYPES[s["type"]],
                            RATE_SHARE * hz if hz else DEFAULT_MIN_HZ, s["frame"],
                            f"sensor {name} ({s['type']})"))
    return out


def check_topic(e, o):
    """Problems with one topic, [] when it passes."""
    # `type`: what the graph has on the topic (comma separated), empty when nobody publishes it.
    if o is not None and o.type and o.type != e.type:
        return [f"{e.topic}: type {o.type}, expected {e.type}"]
    if o is None or not o.samples:
        return [f"{e.topic}: nothing arrived ({e.what})"]
    problems = []
    stamps = [s for s, _ in o.samples]
    if any(b < a for a, b in zip(stamps, stamps[1:])):
        problems.append(f"{e.topic}: stamps go back")
    span = stamps[-1] - stamps[0]
    hz = (len(stamps) - 1) / span if span > 0 else 0.0
    if hz < e.min_hz:
        problems.append(f"{e.topic}: {hz:.1f} Hz on its stamps, expected at least "
                        f"{e.min_hz:.1f}")
    if e.frame is not None:
        frames = {f for _, f in o.samples}
        if frames != {e.frame}:
            problems.append(f"{e.topic}: frame_id {sorted(frames)}, expected {e.frame!r}")
    if o.clock_at_arrival:
        worst = max(abs(c - s) for (s, _), c in zip(o.samples, o.clock_at_arrival)
                    if c is not None)
        if worst > CLOCK_SLACK:
            problems.append(f"{e.topic}: a stamp {worst:.2f} s from /clock (the simulation "
                            "clock the stamps must be on)")
    return problems


def check_description(spec, simulator, urdf):
    """Problems with /robot_description: every published sensor's link and frame in it."""
    if not urdf:
        return ["/robot_description: nothing arrived (start the car's vehicle description)"]
    problems = []
    for name, s in sensors_for(spec, simulator).items():
        for link in dict.fromkeys((s["link"], s["frame"])):
            if f'<link name="{link}"' not in urdf:
                problems.append(f"/robot_description: no link {link!r} (sensor {name})")
    for link in ("base_link", "sensor_kit_base_link"):
        if f'<link name="{link}"' not in urdf:
            problems.append(f"/robot_description: no link {link!r}")
    return problems


def check(spec, simulator, observed, urdf):
    """{topic or /robot_description: [problems]} for everything the contract names."""
    out = {e.topic: check_topic(e, observed.get(e.topic)) for e in expected(spec, simulator)}
    out["/robot_description"] = check_description(spec, simulator, urdf)
    return out
