"""The dynamics run: what the car does with Autoware's commands, against what they ask.
Pure: the plan, and the analysis of what was recorded.

The run drives the vehicle side's control interface directly (/control/command/control_cmd,
as Autoware's controller does): acceleration steps from rest to SPEED, braking steps from
SPEED to rest, and a steering sweep at STEER_SPEED alternating left and right so the car stays
near its lane. Each step is judged on its steady part: the acceleration the car reached (a
line through its speeds) against the request, within acceptance.py's 0.35 m/s^2; the steering
angle the car reports against the command; and the yaw rate against what the angle gives on
the spec's wheel base (v tan(delta) / L), within YAW_SHARE.
"""
import math
from dataclasses import dataclass

SPEED = 6.0                   # m/s, the speed accelerations run to and brakings from
STEER_SPEED = 4.0             # m/s, the steering sweep's
ACCELS = (1.0, 2.0, 3.0)      # m/s^2
BRAKES = (-1.0, -2.5, -4.0)   # m/s^2
STEERS = (0.05, 0.10, 0.15)   # rad, tire angle
STEER_SECONDS = 2.5           # per side
TRACKING = 0.35               # m/s^2, acceptance.py's longitudinal limit
STEER_TOLERANCE = 0.02        # rad
YAW_SHARE = 0.15              # the yaw rate's share it may be off the bicycle model's
SETTLE = 0.8                  # s at the start of a step that is not judged (the lags)


@dataclass
class Step:
    name: str
    kind: str          # accel, brake, steer
    request: float     # m/s^2 or rad


def plan():
    steps = []
    for a, b in zip(ACCELS, BRAKES):
        steps.append(Step(f"accel {a:+.1f}", "accel", a))
        steps.append(Step(f"brake {b:+.1f}", "brake", b))
    for d in STEERS:
        steps.append(Step(f"steer {d:+.2f}", "steer", d))
        steps.append(Step(f"steer {-d:+.2f}", "steer", -d))
    return steps


def command(step, t_in_step, speed):
    """(acceleration, target velocity, tire angle) for `step`, `t_in_step` s into it."""
    if step.kind == "accel":
        return step.request, SPEED + 10.0, 0.0
    if step.kind == "brake":
        return step.request, 0.0, 0.0
    # steer: hold STEER_SPEED with a P controller on the acceleration
    a = max(-2.0, min(2.0, 1.5 * (STEER_SPEED - speed)))
    return a, STEER_SPEED, step.request


def done(step, t_in_step, speed):
    if step.kind == "accel":
        return speed >= SPEED or t_in_step > 20.0
    if step.kind == "brake":
        return speed <= 0.2 or t_in_step > 20.0
    return t_in_step >= STEER_SECONDS


def slope(ts, vs):
    """The least-squares line's slope through (t, v)."""
    n = len(ts)
    if n < 3:
        return None
    mt, mv = sum(ts) / n, sum(vs) / n
    den = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (v - mv) for t, v in zip(ts, vs)) / den if den > 0 else None


def judge(step, samples, wheel_base):
    """{step, request, measured, error, pass, ...} from a step's samples: dicts with t (s
    into the step), v, steer (the reported tire angle), yaw_rate."""
    out = {"step": step.name, "request": step.request}
    if step.kind in ("accel", "brake"):
        lo, hi = 0.5, SPEED - 0.5
        win = [s for s in samples if s["t"] >= SETTLE and lo <= s["v"] <= hi]
        a = slope([s["t"] for s in win], [s["v"] for s in win])
        if a is None:
            return {**out, "pass": False, "why": f"{len(win)} samples in its steady part"}
        err = a - step.request
        return {**out, "measured": round(a, 3), "error": round(err, 3),
                "pass": abs(err) <= TRACKING}
    win = [s for s in samples if s["t"] >= STEER_SECONDS / 2]
    if len(win) < 3:
        return {**out, "pass": False, "why": f"{len(win)} samples in its steady part"}
    steer = sum(s["steer"] for s in win) / len(win)
    v = sum(s["v"] for s in win) / len(win)
    yaw = sum(s["yaw_rate"] for s in win) / len(win)
    want = v * math.tan(step.request) / wheel_base
    share = (yaw - want) / want if want else float("inf")
    ok = abs(steer - step.request) <= STEER_TOLERANCE and abs(share) <= YAW_SHARE
    return {**out, "steer": round(steer, 4), "speed": round(v, 2), "yaw_rate": round(yaw, 4),
            "bicycle_yaw_rate": round(want, 4), "yaw_share_off": round(share, 3), "pass": ok}
