"""The dynamics run's plan and judgement, on made-up recordings."""
import math

from autoware_sim_check import dynamics as D


def ramp(a, v0=0.0, dt=0.05, until=None, lag=0.0):
    out, t, v = [], 0.0, v0
    while t < 12.0:
        out.append({"t": t, "v": v, "steer": 0.0, "yaw_rate": 0.0})
        if t >= lag:
            v = max(0.0, v + a * dt)
        t += dt
        if until and until(v):
            break
    return out


def test_the_plan_pairs_accelerations_and_brakings_then_sweeps_both_ways():
    names = [s.name for s in D.plan()]
    assert names[:2] == ["accel +1.0", "brake -1.0"]
    assert names[-2:] == ["steer +0.15", "steer -0.15"]


def test_a_car_that_does_what_it_is_asked_passes():
    step = D.Step("accel +2.0", "accel", 2.0)
    r = D.judge(step, ramp(2.0, lag=0.3, until=lambda v: v >= D.SPEED), 3.005)
    assert r["pass"] and abs(r["error"]) < 0.01


def test_a_car_short_of_the_request_fails_with_its_error():
    step = D.Step("brake -4.0", "brake", -4.0)
    r = D.judge(step, ramp(-3.0, v0=D.SPEED, until=lambda v: v <= 0.2), 3.005)
    assert not r["pass"] and abs(r["error"] - 1.0) < 0.01


def test_the_steering_is_judged_on_angle_and_yaw_rate():
    step = D.Step("steer +0.10", "steer", 0.10)
    good = math.tan(0.10) * 4.0 / 3.005
    samples = [{"t": 0.05 * i, "v": 4.0, "steer": 0.10, "yaw_rate": good} for i in range(50)]
    assert D.judge(step, samples, 3.005)["pass"]
    for s in samples:
        s["yaw_rate"] = good * 0.7
    r = D.judge(step, samples, 3.005)
    assert not r["pass"] and abs(r["yaw_share_off"] + 0.3) < 1e-3


def test_commands_hold_the_steering_speed():
    step = D.Step("steer +0.05", "steer", 0.05)
    a, v, d = D.command(step, 0.0, 2.0)
    assert a == 2.0 and v == D.STEER_SPEED and d == 0.05
    assert D.done(step, D.STEER_SECONDS, 4.0)


# A small car (AutoSDV's 1/10 car A twin, top speed ~2 m/s) runs the same steps below its top
# speed; a road car keeps the plan's own speeds.

SMALL_EV = {"geometry": {"wheel_radius": 0.052},
            "powertrain": {"model": "ev", "gear_ratio": 8.0, "motor_max_rpm": 3000.0}}


def test_an_ev_spec_has_a_top_speed():
    assert abs(D.top_speed(SMALL_EV) - 3000.0 / 8.0 * 2 * math.pi * 0.052 / 60.0) < 1e-9


def test_a_spec_without_a_motor_limit_has_none():
    assert D.top_speed({"geometry": {"wheel_radius": 0.3}, "powertrain": {"model": "physx"}}) is None


def test_a_road_car_keeps_the_road_speeds():
    for top in (None, 50.0):
        sp = D.speeds_for(top)
        assert (sp.speed, sp.steer_speed, sp.settle, sp.margin) == (D.SPEED, D.STEER_SPEED, D.SETTLE, 0.5)


def test_a_small_car_runs_below_its_top_speed():
    top = D.top_speed(SMALL_EV)
    sp = D.speeds_for(top)
    assert sp.speed <= 0.8 * top + 1e-9 and sp.steer_speed < sp.speed
    assert D.command(D.Step("steer +0.05", "steer", 0.05), 0.0, 0.0, sp)[1] == sp.steer_speed
    assert D.done(D.Step("accel +1.0", "accel", 1.0), 1.0, sp.speed, sp)


def test_a_small_car_that_does_what_it_is_asked_passes():
    sp = D.speeds_for(D.top_speed(SMALL_EV))
    for a in D.ACCELS:
        step = D.Step(f"accel {a:+.1f}", "accel", a)
        r = D.judge(step, ramp(a, lag=0.05, until=lambda v: v >= sp.speed), 0.319, sp)
        assert r["pass"], r
