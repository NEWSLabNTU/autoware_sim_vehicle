"""The contract, on made-up observations of the Model 3's vehicle side."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deps/sim_vehicle_spec"))

from sim_vehicle_spec import spec as specmod  # noqa: E402

from autoware_sim_check import contract as C  # noqa: E402

SPEC = specmod.load(ROOT / "deps/sim_vehicle_spec/vehicles/tesla_model3.yaml")


def stream(type_, hz, frame, n=40, t0=100.0):
    o = C.Observed(type=type_)
    for i in range(n):
        o.samples.append((t0 + i / hz, frame))
        o.clock_at_arrival.append(t0 + i / hz + 0.01)
    return o


def healthy(simulator):
    obs = {}
    for e in C.expected(SPEC, simulator):
        obs[e.topic] = stream(e.type, e.min_hz * 1.3, e.frame or "")
    return obs


URDF = "".join(f'<link name="{n}"/>' for n in (
    "base_link", "sensor_kit_base_link", "top_base_link", "top", "camera6/camera_link",
    "camera6/camera_optical_link", "imu_link", "gnss_link"))


def test_carla_publishes_the_camera_isaac_does_not():
    carla = {e.topic for e in C.expected(SPEC, "carla")}
    isaac = {e.topic for e in C.expected(SPEC, "isaac")}
    assert "/sensing/camera/camera6/image_raw" in carla - isaac
    assert "/sensing/lidar/top/pointcloud_before_sync" in carla & isaac


def test_a_healthy_vehicle_side_passes():
    result = C.check(SPEC, "carla", healthy("carla"), URDF)
    assert all(not p for p in result.values()), result


def test_each_fault_is_named():
    obs = healthy("carla")
    lidar = "/sensing/lidar/top/pointcloud_before_sync"
    obs[lidar] = stream("sensor_msgs/msg/PointCloud2", 5.0, "top")
    obs["/vehicle/status/velocity_status"] = stream(
        "autoware_vehicle_msgs/msg/VelocityReport", 30.0, "map")
    del obs["/sensing/imu/imu_link/imu_raw"]
    gnss = obs["/sensing/gnss/gnss_link/nav_sat_fix"]
    gnss.clock_at_arrival = [c + 5.0 for c in gnss.clock_at_arrival]
    r = C.check(SPEC, "carla", obs, URDF.replace('<link name="imu_link"/>', ""))
    assert "5.0 Hz" in r[lidar][0]
    assert "frame_id ['map']" in r["/vehicle/status/velocity_status"][0]
    assert "nothing arrived" in r["/sensing/imu/imu_link/imu_raw"][0]
    assert "from /clock" in r["/sensing/gnss/gnss_link/nav_sat_fix"][0]
    assert r["/robot_description"] == ["/robot_description: no link 'imu_link' (sensor imu)"]
