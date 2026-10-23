"""vehicle_check: a simulated car's vehicle side against its vehicle spec, with no Autoware
and no scenario (roadmap 051).

    ros2 run autoware_sim_check vehicle_check [contract|dynamics|all] --vehicle-model tesla_model3
        [--simulator carla|isaac|any] [--listen 10] [--out report.json]

Start the vehicle side first (acb_bridge with a hero in CARLA, aib's runner in Isaac) and the
car's vehicle description, which `vehicle_check.launch.xml` does with this node. `contract`
listens for --listen seconds of simulation time and checks the topics; `dynamics` then drives
the car with Autoware's control command (the car must stand on a flat straight road with
about 150 m ahead, or a ground plane) and judges each step. Exit 0 when everything passes.
The report (--out) has every topic's problems and every step's numbers.
"""
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       qos_profile_sensor_data)

from autoware_sim_check import contract as C
from autoware_sim_check import dynamics as D


# A step's limit in wall time: a simulator at a fraction of real time still finishes.
WALL_PER_STEP = 120.0


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def load_spec(vehicle_model):
    from ament_index_python.packages import get_package_share_directory
    from sim_vehicle_spec import spec as specmod

    share = get_package_share_directory("sim_vehicle_spec")
    return specmod.load(f"{share}/vehicles/{vehicle_model}.yaml")


class Check(Node):
    def __init__(self, spec, simulator):
        super().__init__("vehicle_check", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        from rosgraph_msgs.msg import Clock
        from rosidl_runtime_py.utilities import get_message
        from std_msgs.msg import String

        self.spec, self.simulator = spec, simulator
        self.clock = None
        self.urdf = ""
        self.observed = {}
        self.vel = None          # (stamp, longitudinal, heading_rate)
        self.steer = 0.0
        self.recording = None    # list the samples go to while a step runs
        # Best effort takes /clock from a reliable publisher and from a best-effort one.
        self.create_subscription(Clock, "/clock", self.on_clock, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "/robot_description", self.on_urdf, latched)
        for e in C.expected(spec, simulator):
            self.observed[e.topic] = C.Observed()
            cls = get_message(e.type)
            self.create_subscription(cls, e.topic, self.on_message(e.topic),
                                     qos_profile_sensor_data)

    def on_clock(self, msg):
        self.clock = stamp_s(msg.clock)

    def on_urdf(self, msg):
        self.urdf = msg.data

    def on_message(self, topic):
        def cb(msg):
            header = getattr(msg, "header", None)
            if header is not None:
                t, frame = stamp_s(header.stamp), header.frame_id
            else:
                t, frame = stamp_s(msg.stamp), ""
            o = self.observed[topic]
            if self.listening:
                o.samples.append((t, frame))
                o.clock_at_arrival.append(self.clock)
            if topic == "/vehicle/status/velocity_status":
                self.vel = (t, msg.longitudinal_velocity, msg.heading_rate)
                self.sample()
            elif topic == "/vehicle/status/steering_status":
                self.steer = msg.steering_tire_angle
        return cb

    listening = False

    def sample(self):
        if self.recording is None or self.vel is None:
            return
        self.recording.append({"stamp": self.vel[0], "v": self.vel[1], "steer": self.steer,
                               "yaw_rate": self.vel[2]})

    def spin_for(self, seconds, sim=True):
        """Spin until `seconds` of simulation time (or wall time) went by."""
        start_wall, start = time.monotonic(), self.clock
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.02)
            if sim and start is not None and self.clock is not None:
                if self.clock - start >= seconds:
                    return
            elif time.monotonic() - start_wall >= seconds:
                return

    def contract(self, listen):
        log = self.get_logger()
        log.info("waiting for /clock and /vehicle/status/velocity_status")
        deadline = time.monotonic() + 120
        while (self.clock is None or self.vel is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.listening = True
        self.spin_for(listen)
        self.listening = False
        # The types the graph has on each topic: a publisher of another type never delivers to
        # our subscription, so without this it would read as "nothing arrived".
        graph = dict(self.get_topic_names_and_types())
        for topic, o in self.observed.items():
            o.type = ",".join(graph.get(topic, []))
        return C.check(self.spec, self.simulator, self.observed, self.urdf)

    def dynamics(self):
        from autoware_control_msgs.msg import Control
        from autoware_vehicle_msgs.msg import GearCommand

        log = self.get_logger()
        deadline = time.monotonic() + 120
        while (self.clock is None or self.vel is None) and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.clock is None or self.vel is None:
            return [{"step": "start", "pass": False,
                     "why": "no /clock or /vehicle/status/velocity_status: no vehicle side"}]
        control = self.create_publisher(Control, "/control/command/control_cmd", 10)
        gear = self.create_publisher(GearCommand, "/control/command/gear_cmd", 10)
        L = self.spec.doc["geometry"]["wheel_base"]

        def send(a, v, delta):
            now = self.get_clock().now().to_msg()
            m = Control()
            m.stamp = m.control_time = now
            m.longitudinal.stamp = m.lateral.stamp = now
            m.longitudinal.velocity = float(v)
            m.longitudinal.acceleration = float(a)
            m.lateral.steering_tire_angle = float(delta)
            control.publish(m)
            g = GearCommand()
            g.stamp = now
            g.command = GearCommand.DRIVE
            gear.publish(g)

        # Stand still first: the stop the run starts from.
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            send(-1.5, 0.0, 0.0)
            self.spin_for(0.03, sim=False)
        results = []
        for step in D.plan():
            log.info(f"step {step.name}")
            self.recording = []
            t0 = self.clock
            # A simulator whose clock stopped ends the step, and the run with it.
            wall_end = time.monotonic() + WALL_PER_STEP
            while rclpy.ok():
                v = self.vel[1] if self.vel else 0.0
                t = (self.clock or t0) - t0
                if D.done(step, t, v):
                    break
                if time.monotonic() > wall_end:
                    results.append({"step": step.name, "pass": False,
                                    "why": f"{WALL_PER_STEP:.0f} s of wall time, {t:.1f} s of "
                                           "simulation time: the simulator stopped"})
                    self.recording = None
                    return results
                send(*D.command(step, t, v))
                self.spin_for(0.03, sim=False)
            samples = [dict(s, t=s["stamp"] - t0) for s in self.recording]
            self.recording = None
            r = D.judge(step, samples, L)
            log.info(json.dumps(r))
            results.append(r)
        t_end = time.monotonic() + 3.0
        while time.monotonic() < t_end:
            send(-2.0, 0.0, 0.0)
            self.spin_for(0.03, sim=False)
        return results


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ros_args = rclpy.utilities.remove_ros_args(["x"] + argv)[1:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("what", nargs="?", default="all", choices=("contract", "dynamics", "all"))
    ap.add_argument("--vehicle-model", default="tesla_model3")
    ap.add_argument("--simulator", default="any", choices=("carla", "isaac", "any"))
    ap.add_argument("--listen", type=float, default=10.0, help="s of simulation time")
    ap.add_argument("--out", default="")
    args = ap.parse_args(ros_args)
    rclpy.init(args=argv)
    node = Check(load_spec(args.vehicle_model), args.simulator)
    report = {"vehicle_model": args.vehicle_model, "simulator": args.simulator}
    ok = True
    try:
        if args.what in ("contract", "all"):
            c = node.contract(args.listen)
            report["contract"] = c
            for topic, problems in c.items():
                print(f"{'PASS' if not problems else 'FAIL'}  {topic}")
                for p in problems:
                    print(f"      {p}")
            ok &= all(not p for p in c.values())
        if args.what in ("dynamics", "all"):
            d = node.dynamics()
            report["dynamics"] = d
            for r in d:
                print(f"{'PASS' if r['pass'] else 'FAIL'}  {json.dumps(r)}")
            ok &= all(r["pass"] for r in d)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.out:
        with open(args.out, "w") as f:
            json.dump(report, f, indent=1)
    print("vehicle_check: " + ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
