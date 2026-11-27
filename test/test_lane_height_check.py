"""lane_height_check (scensim issue 180, option 3, I2): a raw lidar point on a road lanelet that
stands more than h above the map's road is an obstacle point; groups of them that persist are
published for the motion velocity planner, whose obstacle_stop stops for them. These tests
use synthetic maps and clouds, no ROS, except the node test at the end."""

import importlib.machinery
import importlib.util
import math
import textwrap
from pathlib import Path

import numpy as np
import pytest

PROFILE = Path(__file__).resolve().parent.parent / "autoware_sim_profile"


def module():
    path = PROFILE / "scripts" / "lane_height_check"
    loader = importlib.machinery.SourceFileLoader("lane_height_check", str(path))
    spec = importlib.util.spec_from_loader("lane_height_check", loader)
    m = importlib.util.module_from_spec(spec)
    loader.exec_module(m)
    return m


L = module()


# --- synthetic maps --------------------------------------------------------------------------

def write_osm(path, lanelets):
    """lanelets: [(subtype, left [(x, y)], right [(x, y)])] in local coordinates."""
    nodes, ways, rels, nid = [], [], [], 1
    for k, (subtype, left, right) in enumerate(lanelets):
        ids = []
        for line in (left, right):
            refs = []
            for x, y in line:
                nodes.append(f'<node id="{nid}" lat="0" lon="0"><tag k="local_x" v="{x}"/>'
                             f'<tag k="local_y" v="{y}"/><tag k="ele" v="0"/></node>')
                refs.append(nid)
                nid += 1
            wid = 100000 + len(ways)
            ways.append(f'<way id="{wid}">' + "".join(f'<nd ref="{r}"/>' for r in refs)
                        + '<tag k="type" v="line_thin"/></way>')
            ids.append(wid)
        rels.append(f'<relation id="{200000 + k}"><member type="way" role="left" ref="{ids[0]}"/>'
                    f'<member type="way" role="right" ref="{ids[1]}"/>'
                    f'<tag k="type" v="lanelet"/><tag k="subtype" v="{subtype}"/></relation>')
    path.write_text('<?xml version="1.0"?>\n<osm version="0.6">' + "".join(nodes + ways + rels)
                    + "</osm>\n")


def write_pcd(path, pts):
    pts = np.asarray(pts, np.float32)
    head = ("# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            f"WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts)}\nDATA binary\n")
    path.write_bytes(head.encode() + pts.tobytes())


def road(x0, x1, y0, y1, z=lambda x, y: 0.0 * x, step=0.2):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    xs, ys = xs.ravel(), ys.ravel()
    return np.c_[xs, ys, z(xs, ys)]


def straight_map(tmp_path, z=lambda x, y: 0.0 * x, tiles=False):
    """A straight two-lane road along +x, 0 to 100 m, lanes |y| < 3.5, and a walkway beside."""
    xs = [float(x) for x in range(0, 101, 10)]
    write_osm(tmp_path / "lanelet2_map.osm", [
        ("road", [(x, 3.5) for x in xs], [(x, 0.0) for x in xs]),
        ("road", [(x, 0.0) for x in xs], [(x, -3.5) for x in xs]),
        ("walkway", [(x, 6.0) for x in xs], [(x, 3.5) for x in xs]),
    ])
    pts = road(-5, 105, -8, 8, z)
    if tiles:
        d = tmp_path / "pointcloud_map.pcd"
        d.mkdir()
        meta = ["x_resolution: 100.0", "y_resolution: 100.0"]
        cell = np.floor(pts[:, :2] / 100.0).astype(int)
        for i, j in sorted(set(map(tuple, cell))):
            k = (cell[:, 0] == i) & (cell[:, 1] == j)
            write_pcd(d / f"m_{i}_{j}.pcd", pts[k])
            meta.append(f"m_{i}_{j}.pcd: [{i * 100}, {j * 100}]")
        (tmp_path / "pointcloud_map_metadata.yaml").write_text("\n".join(meta) + "\n")
    else:
        write_pcd(tmp_path / "pointcloud_map.pcd", pts)
    return tmp_path


def box(cx, cy, size, height, z0=0.0, step=0.04):
    """A lidar's view of a box: its top and its near (-x) face, as points."""
    xs, ys = np.meshgrid(np.arange(cx - size / 2, cx + size / 2, step),
                         np.arange(cy - size / 2, cy + size / 2, step))
    top = np.c_[xs.ravel(), ys.ravel(), np.full(xs.size, z0 + height)]
    ys2, zs = np.meshgrid(np.arange(cy - size / 2, cy + size / 2, step),
                          np.arange(z0, z0 + height, step))
    face = np.c_[np.full(ys2.size, cx - size / 2), ys2.ravel(), zs.ravel()]
    return np.r_[top, face]


VEHICLE = {"wheel_base": 3.005, "front_overhang": 0.807, "rear_overhang": 0.98,
           "wheel_tread": 1.667, "left_overhang": 0.248, "right_overhang": 0.248}


def checker(map_dir, clearance=0.138):
    return L.LaneHeightCheck(str(map_dir), L.Settings.from_vehicle(VEHICLE, clearance))


POSE0 = (0.0, 0.0, 0.0, 0.0)  # x, y, z, yaw of base_link in the map


# --- the threshold ---------------------------------------------------------------------------

def test_the_threshold_is_the_clearance_less_a_margin_with_a_floor():
    assert L.threshold(0.138) == pytest.approx(0.098)
    assert L.threshold(0.03) == pytest.approx(0.05)     # MuSHR nano: the floor
    assert L.threshold(0.09) == pytest.approx(0.05)
    assert L.threshold(0.20, margin=0.05, floor=0.05) == pytest.approx(0.15)


# --- the lanelets ----------------------------------------------------------------------------

def test_only_road_lanelets_are_the_carriageway(tmp_path):
    straight_map(tmp_path)
    lanes = L.Lanes.from_osm(str(tmp_path / "lanelet2_map.osm"))
    inside = lanes.contains(np.array([[50.0, 1.0], [50.0, -3.0], [50.0, 5.0], [120.0, 0.0]]))
    assert inside.tolist() == [True, True, False, False]


def test_a_curved_lanelet_holds_its_arc_not_its_chord(tmp_path):
    a = np.radians(np.arange(0, 91, 5))
    left = [(30 * math.cos(t), 30 * math.sin(t)) for t in a]
    right = [(26.5 * math.cos(t), 26.5 * math.sin(t)) for t in a]
    write_osm(tmp_path / "lanelet2_map.osm", [("road", left, right)])
    lanes = L.Lanes.from_osm(str(tmp_path / "lanelet2_map.osm"))
    on_arc = (28.25 * math.cos(math.radians(45)), 28.25 * math.sin(math.radians(45)))
    on_chord = (0.5 * (28.25 + 0.0), 0.5 * (0.0 + 28.25))   # inside the chord, off the road
    assert lanes.contains(np.array([on_arc, on_chord])).tolist() == [True, False]


# --- the height above the map ----------------------------------------------------------------

def test_a_flat_road_makes_no_obstacle_and_a_15_cm_box_does(tmp_path):
    c = checker(straight_map(tmp_path))
    cloud = np.r_[road(2, 30, -3, 3), box(15.0, 1.0, 0.3, 0.15)]
    pts = c.obstacle_points(cloud, POSE0)
    assert len(pts) > 0
    assert np.all(np.abs(pts[:, 0] - 15.0) < 0.3) and np.all(np.abs(pts[:, 1] - 1.0) < 0.3)


def test_an_8_cm_box_is_under_the_model3s_threshold(tmp_path):
    c = checker(straight_map(tmp_path))
    assert len(c.obstacle_points(box(15.0, 1.0, 0.3, 0.08), POSE0)) == 0
    # with the 0.05 m floor (a low car) it is an obstacle
    low = checker(tmp_path, clearance=0.03)
    assert len(low.obstacle_points(box(15.0, 1.0, 0.3, 0.08), POSE0)) > 0


def test_a_grade_and_a_crown_are_not_obstacles(tmp_path):
    z = lambda x, y: 0.10 * x - 0.02 * np.abs(y)   # 10 % grade, 2 % crown
    c = checker(straight_map(tmp_path, z))
    cloud = road(2, 30, -3.4, 3.4, z)
    assert len(c.obstacle_points(cloud, POSE0)) == 0
    rock = box(15.0, 1.0, 0.3, 0.15, z0=float(z(15.0, 1.0)))
    assert len(c.obstacle_points(np.r_[cloud, rock], POSE0)) > 0


def test_points_off_the_lanelets_are_ignored(tmp_path):
    c = checker(straight_map(tmp_path))
    assert len(c.obstacle_points(box(15.0, 5.0, 0.3, 0.30), POSE0)) == 0     # on the walkway
    assert len(c.obstacle_points(box(15.0, 9.0, 0.3, 0.30), POSE0)) == 0     # off the map


def test_the_range_and_the_ego_box(tmp_path):
    c = checker(straight_map(tmp_path))
    assert len(c.obstacle_points(box(35.0, 1.0, 0.3, 0.3), POSE0)) == 0      # beyond 30 m
    assert len(c.obstacle_points(box(2.0, 0.0, 0.3, 0.3), POSE0)) == 0       # the ego's own body
    assert len(c.obstacle_points(box(-5.0, 1.0, 0.3, 0.3), POSE0)) == 0      # behind


def test_the_cloud_is_placed_by_the_pose(tmp_path):
    c = checker(straight_map(tmp_path))
    # base_link at x 40 heading +x, z 0.5 above the map frame's road (a cloud in base_link)
    pose = (40.0, 1.0, 0.0, 0.0)
    rock = box(10.0, 0.0, 0.3, 0.15)                # 10 m ahead of the car, at map (50, 1)
    pts = c.obstacle_points(rock, pose)
    assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 50.0) < 0.3)
    yawed = (40.0, 1.0, 0.0, math.pi)               # facing -x: 10 m ahead is map x 30
    pts = c.obstacle_points(rock, yawed)
    assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 30.0) < 0.3)


def test_a_divided_map_reads_its_tiles(tmp_path):
    c = checker(straight_map(tmp_path, tiles=True))
    assert len(c.obstacle_points(box(15.0, 1.0, 0.3, 0.15), POSE0)) > 0
    assert len(c.obstacle_points(road(2, 30, -3, 3), POSE0)) == 0


# --- detections and persistence --------------------------------------------------------------

def test_a_detection_needs_n_points():
    pts = np.array([[10.0, 1.0, 0.2], [10.1, 1.0, 0.2]])
    assert L.detections(pts, cell=0.3, min_points=3) == []
    pts = np.r_[pts, [[10.2, 1.1, 0.2]]]
    assert len(L.detections(pts, cell=0.3, min_points=3)) == 1


def test_touching_cells_are_one_detection_and_apart_are_two():
    a = np.array([[10.0, 1.0, 0.2]] * 3 + [[10.35, 1.0, 0.2]] * 3)
    assert len(L.detections(a, cell=0.3, min_points=3)) == 1
    b = np.array([[10.0, 1.0, 0.2]] * 3 + [[12.0, 1.0, 0.2]] * 3)
    assert len(L.detections(b, cell=0.3, min_points=3)) == 2


def test_a_detection_is_published_from_its_third_consecutive_frame():
    t = L.Persistence(frames=3, radius=1.0)
    rock = np.array([[10.0, 1.0, 0.2]] * 3)
    assert len(t.update([rock])) == 0
    assert len(t.update([rock + [0.1, 0, 0]])) == 0
    assert len(t.update([rock + [0.2, 0, 0]])) == 3        # its points
    assert len(t.update([])) == 0                          # gone: dropped at once
    assert len(t.update([rock])) == 0                      # and starts over


def test_a_flicker_elsewhere_is_never_published():
    t = L.Persistence(frames=3, radius=1.0)
    out = [len(t.update([np.array([[10.0 + 5 * k, 1.0, 0.2]] * 3)])) for k in range(6)]
    assert out == [0] * 6


# --- the node --------------------------------------------------------------------------------

def test_the_node_publishes_a_persistent_rock(tmp_path):
    rclpy = pytest.importorskip("rclpy")
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    straight_map(tmp_path)
    vi = tmp_path / "vehicle_info.param.yaml"
    vi.write_text("/**:\n  ros__parameters:\n" + "".join(
        f"    {k}: {v}\n" for k, v in VEHICLE.items()))
    body = tmp_path / "body.param.yaml"
    body.write_text("/**:\n  ros__parameters:\n    ground_clearance: 0.138\n")
    rclpy.init(args=["--ros-args", "-p", f"map_path:={tmp_path}",
                     "--params-file", str(vi), "--params-file", str(body)])
    try:
        node = L.Node()
        peer = rclpy.create_node("lane_height_check_test")
        got = []
        peer.create_subscription(PointCloud2, "/lane_height_check/output/pointcloud",
                                 got.append, 10)
        pub_c = peer.create_publisher(PointCloud2, "/lane_height_check/input/pointcloud", 10)
        pub_o = peer.create_publisher(Odometry, "/lane_height_check/input/kinematic_state", 10)
        cloud = np.r_[road(2, 30, -3, 3), box(15.0, 1.0, 0.3, 0.15)].astype(np.float32)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(peer)
        for k in range(30):
            stamp = peer.get_clock().now().to_msg()
            o = Odometry()
            o.header.stamp, o.header.frame_id = stamp, "map"
            o.pose.pose.orientation.w = 1.0
            pub_o.publish(o)
            pub_c.publish(L.to_cloud(cloud, stamp, "base_link"))
            for _ in range(5):
                ex.spin_once(timeout_sec=0.02)
        sizes = [m.width * m.height for m in got]
        assert sizes and max(sizes) > 0, sizes
        assert got[-1].header.frame_id == "base_link"
        out = L.from_cloud(got[-1])
        assert np.all(np.abs(out[:, 0] - 15.0) < 0.3)
    finally:
        rclpy.shutdown()


def test_the_launch_wires_only_the_motion_velocity_planner():
    text = (PROFILE / "launch/autoware.launch.xml").read_text()
    assert '<arg name="lane_height_check" default="false"' in text
    assert ('<set_remap from="motion_velocity_planner:~/input/no_ground_pointcloud" '
            'to="/perception/obstacle_segmentation/lane_height_check/pointcloud"') in text
    assert 'exec="lane_height_check"' in text


def test_obstacle_stop_copy_differs_from_upstream_only_in_the_pointcloud_check():
    import yaml
    up = Path("/opt/autoware/1.5.0/share/autoware_launch/config/planning/scenario_planning/"
              "lane_driving/motion_planning/motion_velocity_planner/obstacle_stop.param.yaml")
    if not up.exists():
        pytest.skip("no upstream autoware_launch 1.5.0")
    ours = yaml.safe_load((PROFILE / "config/planning/motion_velocity_planner/"
                           "obstacle_stop.lane_height_check.param.yaml").read_text())
    theirs = yaml.safe_load(up.read_text())
    f = ours["/**"]["ros__parameters"]["obstacle_stop"]["obstacle_filtering"]
    assert f["check_inside"]["pointcloud"] is True
    f["check_inside"]["pointcloud"] = False
    assert ours == theirs
