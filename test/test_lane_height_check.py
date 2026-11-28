"""lane_height_check (scensim issue 180, option 3, I2): a raw lidar point on a road lanelet that
stands more than h above the map's road is an obstacle point; groups of them that persist are
published for the motion velocity planner, whose obstacle_stop stops for them. These tests
use synthetic maps and clouds, no ROS, except the node test at the end."""

import importlib.machinery
import importlib.util
import math
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


def quat(yaw=0.0, pitch=0.0, roll=0.0):
    """(qx, qy, qz, qw) of a ZYX rotation: yaw about z, then pitch about y, then roll about x
    (ROS: a positive pitch is nose down)."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def rot(yaw=0.0, pitch=0.0, roll=0.0):
    """The same rotation as a matrix, built here apart from the node's code."""
    cy, sy, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return rz @ ry @ rx


def pose(x=0.0, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0):
    """base_link in the map as the node takes it from the EKF: position, then quaternion."""
    return (x, y, z) + quat(yaw, pitch, roll)


def seen_from(pts_map, x=0.0, y=0.0, z=0.0, yaw=0.0, pitch=0.0, roll=0.0):
    """Map points as the car's lidar gives them: in base_link, which tilts with the car."""
    return (np.asarray(pts_map, float) - [x, y, z]) @ rot(yaw, pitch, roll)


POSE0 = pose()  # base_link at the map's origin, level, heading +x


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


def test_a_car_pitched_on_a_grade_sees_no_obstacle_on_it(tmp_path):
    """Scensim issue 201: base_link tilts with the car, so on a grade the lidar sees the road
    ahead about level; placed by yaw alone, a downhill road rose r sin(pitch) above the map
    and became an obstacle from about 1 m ahead. Uphill, a rock had to stand that much higher."""
    for g in (0.10, -0.10):                         # uphill, downhill along +x
        tmp = tmp_path / f"g{g}"
        tmp.mkdir()
        z = lambda x, y, g=g: g * x
        c = checker(straight_map(tmp, z))
        car = dict(x=10.0, y=1.0, z=10.0 * g, pitch=-math.atan(g))   # on the road, nose with it
        ahead = road(12, 40, -3.4, 3.4, z)
        assert len(c.obstacle_points(seen_from(ahead, **car), pose(**car))) == 0, g
        rock = box(25.0, 1.0, 0.3, 0.15, z0=float(z(25.0, 1.0)))
        pts = c.obstacle_points(seen_from(np.r_[ahead, rock], **car), pose(**car))
        assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 25.0) < 0.3), g


def test_a_car_rolled_on_a_cross_slope_sees_no_obstacle_on_it(tmp_path):
    z = lambda x, y: 0.05 * y                       # 5 % across, rising to the left
    c = checker(straight_map(tmp_path, z))
    car = dict(x=10.0, y=1.0, z=0.05, roll=math.atan(0.05))
    ahead = road(12, 40, -3.4, 3.4, z)
    assert len(c.obstacle_points(seen_from(ahead, **car), pose(**car))) == 0
    rock = box(25.0, -2.0, 0.3, 0.15, z0=float(z(25.0, -2.0)))
    assert len(c.obstacle_points(seen_from(np.r_[ahead, rock], **car), pose(**car))) > 0


def test_a_turned_car_on_a_grade_is_placed_by_the_whole_rotation(tmp_path):
    """Yaw, then pitch: a car heading -x down a road that falls along -x (so uphill behind it)
    is nose down; the order of the rotations decides where its points land."""
    z = lambda x, y: 0.08 * x
    c = checker(straight_map(tmp_path, z))
    car = dict(x=60.0, y=-1.0, z=4.8, yaw=math.pi, pitch=math.atan(0.08))
    ahead = road(20, 58, -3.4, 3.4, z)
    assert len(c.obstacle_points(seen_from(ahead, **car), pose(**car))) == 0
    rock = box(45.0, -1.0, 0.3, 0.15, z0=float(z(45.0, -1.0)))
    pts = c.obstacle_points(seen_from(np.r_[ahead, rock], **car), pose(**car))
    assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 45.0) < 0.3)


def test_points_return_to_base_link_by_the_inverse_rotation():
    """The published cloud is in base_link with the input's stamp: the map points go back by
    the same pose, pitched and rolled too."""
    car = dict(x=5.0, y=-2.0, z=0.7, yaw=0.6, pitch=0.05, roll=-0.03)
    local = np.array([[12.0, 1.0, 0.2], [20.0, -0.5, 0.1]])
    m = L.to_map(local, pose(**car))
    assert np.allclose(m, local @ rot(car["yaw"], car["pitch"], car["roll"]).T
                       + [car["x"], car["y"], car["z"]])
    assert np.allclose(L.to_base_link(m, pose(**car)), local)


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
    rock = box(10.0, 0.0, 0.3, 0.15)                # 10 m ahead of the car, at map (50, 1)
    pts = c.obstacle_points(rock, pose(40.0, 1.0))
    assert len(pts) > 0 and np.all(np.abs(pts[:, 0] - 50.0) < 0.3)
    yawed = pose(40.0, 1.0, yaw=math.pi)            # facing -x: 10 m ahead is map x 30
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


# --- holding a detection the ego mask hides (scensim issue 180, R180_B1) --------------------
#
# The node drops every point inside the ego box grown by EGO_MARGIN, its own body, and the roof
# lidar cannot see low points just past the bumper (the bonnet's shadow). A rock the car stops
# that close to disappears, obstacle_stop's pointcloud hold runs out, and the car drives over it.
# A published detection that vanishes just ahead of the car is held at its map position instead.

FRONT = 3.005 + 0.807            # VEHICLE's front bumper from base_link
SHADOW = 1.2                     # m past the bumper the lidar cannot see low points (the bonnet's
                                 # shadow: issue 182's blind zone ends 4.9 m from base_link)


def lidar_sees(local):
    """A base_link cloud as the roof lidar returns it: no low point in the bonnet's shadow."""
    local = np.asarray(local, float).reshape(-1, 3)
    hidden = (local[:, 0] > 0) & (local[:, 0] < FRONT + SHADOW) & (local[:, 2] < 0.5)
    return local[~hidden]


def rock_at(x, y, height=0.15):
    return box(x, y, 0.3, height)


def drive(c, poses, scene, frames_per_pose=1):
    """The node's loop over `poses` (pose() tuples), the lidar seeing `scene` (map points) from
    each; returns the published map points per frame."""
    persist, hold = L.Persistence(), L.MaskHold(c.s)
    out = []
    for p in poses:
        car = dict(zip(("x", "y", "z"), p[:3]))
        q = p[3:]
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q[3] * q[1] - q[2] * q[0]))))
        roll = math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 1 - 2 * (q[0] ** 2 + q[1] ** 2))
        cloud = lidar_sees(seen_from(scene, yaw=yaw, pitch=pitch, roll=roll, **car))
        for _ in range(frames_per_pose):
            out.append(L.step(c, persist, hold, cloud, p)[2])
    return out


def near(pts, x, y, r=0.3):
    return len(pts) > 0 and bool(np.all(np.hypot(pts[:, 0] - x, pts[:, 1] - y) < r))


def approach(rock_x, gaps, y=1.0):
    """base_link poses whose bumper stands `gaps` before the rock's near face (0.15 m before
    its centre)."""
    return [pose(rock_x - 0.15 - g - FRONT, y) for g in gaps]


def test_h1_a_rock_the_car_stops_at_stays_published_while_it_stands(tmp_path):
    c = checker(straight_map(tmp_path))
    rock = rock_at(40.0, 1.0)
    poses = approach(40.0, list(np.arange(10.0, 0.0, -0.25)) + [0.06]) + \
        approach(40.0, [0.06] * 100)
    out = drive(c, poses, rock)
    first = next(i for i, o in enumerate(out) if len(o))
    assert first < 10                                       # seen at range, as today
    stand = out[-100:]
    assert all(near(o, 40.0, 1.0) for o in stand), [len(o) for o in stand][:10]


def test_h2_backing_off_hands_the_rock_back_to_the_ordinary_check(tmp_path):
    c = checker(straight_map(tmp_path))
    rock = rock_at(40.0, 1.0)
    gaps = list(np.arange(10.0, 0.0, -0.25)) + [0.06] * 20 + list(np.arange(0.1, 2.6, 0.1))
    out = drive(c, approach(40.0, gaps), rock)
    i0 = next(i for i, o in enumerate(out) if len(o))
    assert all(near(o, 40.0, 1.0) for o in out[i0:]), [len(o) for o in out[i0:]]
    # no double count: the frame after backing off publishes no more points than one view
    # of the rock gives
    x_last = 40.0 - 0.15 - gaps[-1] - FRONT
    one_view = len(L.step(c, L.Persistence(), L.MaskHold(c.s),
                          lidar_sees(seen_from(rock, x=x_last, y=1.0)), pose(x_last, 1.0))[0])
    assert len(out[-1]) <= one_view


def test_h3_a_detection_never_published_is_not_held(tmp_path):
    c = checker(straight_map(tmp_path))
    rock = rock_at(40.0, 1.0)
    # the rock appears 2 frames before the car is on it (fewer than FRAMES): never published
    poses = approach(40.0, [1.0, 0.5, 0.06] + [0.06] * 20)
    scene_frames = [rock, rock] + [np.zeros((0, 3))] * 21
    persist, hold = L.Persistence(), L.MaskHold(c.s)
    out = []
    for p, sc in zip(poses, scene_frames):
        out.append(L.step(c, persist, hold, lidar_sees(seen_from(sc, x=p[0], y=p[1])), p)[2])
    assert all(len(o) == 0 for o in out)


def test_h4_the_cars_own_body_is_never_published(tmp_path):
    c = checker(straight_map(tmp_path))
    # points on the bonnet and the bumper, always inside the mask, every frame
    body = np.array([[FRONT - 0.3, 0.0, 0.6], [FRONT + 0.05, 0.5, 0.3], [FRONT + 0.1, -0.5, 0.25]])
    persist, hold = L.Persistence(), L.MaskHold(c.s)
    for k in range(50):
        p = pose(10.0 + 0.2 * k, 1.0)
        assert len(L.step(c, persist, hold, body, p)[2]) == 0


def test_h5_a_held_rock_behind_the_front_bumper_is_dropped(tmp_path):
    c = checker(straight_map(tmp_path))
    rock = rock_at(40.0, 1.0)
    gaps = list(np.arange(10.0, 0.0, -0.25)) + [0.06] * 10 + [-0.5, -1.0, -2.0, -3.0]
    out = drive(c, approach(40.0, gaps), rock)
    assert near(out[-15], 40.0, 1.0)                        # held at 0.06 m
    assert all(len(o) == 0 for o in out[-3:])               # driven over: gone


def test_h6_a_rock_beside_the_cars_path_is_not_held(tmp_path):
    c = checker(straight_map(tmp_path))
    # 1.25 m left of the car's line: clear of its half width (1.08 m) but inside the mask's
    # margin (1.28 m), so it enters the mask's side as the car passes
    rock = rock_at(40.0, 2.25)
    poses = [pose(x, 1.0) for x in np.arange(20.0, 40.0, 0.4)]
    out = drive(c, poses, rock)
    beside = [(p, o) for p, o in zip(poses, out) if p[0] + FRONT > 40.0 + 0.3]
    assert beside
    for p, o in beside:     # only what the ordinary check still sees outside the mask
        loc = L.to_base_link(o, p)
        inside = (loc[:, 0] < FRONT + L.EGO_MARGIN) & (np.abs(loc[:, 1]) < c.s.half_width + L.EGO_MARGIN)
        assert not inside.any()


def test_h7_held_points_follow_the_whole_rotation(tmp_path):
    g = 0.08
    z = lambda x, y: g * x
    c = checker(straight_map(tmp_path, z))
    rock = rock_at(40.0, 1.0)
    rock[:, 2] += z(40.0, 1.0)
    xs = [40.0 - 0.15 - gap - FRONT for gap in list(np.arange(10.0, 0.0, -0.25)) + [0.06] * 30]
    poses = [pose(x, 1.0, g * x, pitch=-math.atan(g)) for x in xs]
    out = drive(c, poses, rock)
    assert all(near(o, 40.0, 1.0) for o in out[-30:])
    # and placed back into base_link by the same pose, it is where the lidar saw it
    last = L.to_base_link(out[-1], poses[-1])
    assert np.all(last[:, 0] > FRONT) and np.all(last[:, 0] < FRONT + 0.5)


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


def test_the_node_reports_a_status_for_a_cloud_before_its_first_pose(tmp_path):
    """Scensim issue 180's P8: a status for every raw cloud. Before the EKF's first pose the
    node can check nothing, and it says so rather than staying silent."""
    rclpy = pytest.importorskip("rclpy")
    import json
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
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
        peer = rclpy.create_node("lane_height_check_test_nopose")
        got = []
        peer.create_subscription(String, "/lane_height_check/debug/status",
                                 lambda m: got.append(json.loads(m.data)), 10)
        pub_c = peer.create_publisher(PointCloud2, "/lane_height_check/input/pointcloud", 10)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        ex.add_node(peer)
        cloud = road(2, 30, -3, 3).astype(np.float32)
        for k in range(10):
            pub_c.publish(L.to_cloud(cloud, peer.get_clock().now().to_msg(), "base_link"))
            for _ in range(5):
                ex.spin_once(timeout_sec=0.02)
        assert got, "no status before the first pose"
        assert all(s["pose"] is False and s["points"] == len(cloud) for s in got), got[:3]
    finally:
        rclpy.shutdown()


def test_the_launch_wires_only_the_motion_velocity_planner():
    text = (PROFILE / "launch/autoware.launch.xml").read_text()
    assert '<arg name="lane_height_check" default="false"' in text
    assert ('<set_remap from="motion_velocity_planner:~/input/no_ground_pointcloud" '
            'to="$(var lane_height_check_topic)" if="$(var lane_height_check)"/>') in text
    assert ('<let name="lane_height_check_topic" '
            'value="/perception/obstacle_segmentation/lane_height_check/pointcloud"/>') in text
    assert '<remap from="~/output/pointcloud" to="$(var lane_height_check_topic)"/>' in text
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


def test_the_argument_check_refuses_what_lane_height_check_cannot_do():
    loader = importlib.machinery.SourceFileLoader(
        "check_launch_args", str(PROFILE / "scripts" / "check_launch_args"))
    spec = importlib.util.spec_from_loader("check_launch_args", loader)
    m = importlib.util.module_from_spec(spec)
    loader.exec_module(m)
    assert m.problems(["lane_height_check:=true"]) == []
    assert m.problems(["lane_height_check:=false", "autoware_release:=2.0.0"]) == []
    assert "is not one of true, false" in m.problems(["lane_height_check:=yes"])[0]
    p = m.problems(["lane_height_check:=true", "autoware_release:=2.1.0"])
    assert len(p) == 1 and "Autoware 1.5.0" in p[0]
    p = m.problems(["lane_height_check:=true", "obstacle_stop_param_path:=/x.yaml"])
    assert len(p) == 1 and "obstacle_stop_param_path" in p[0]


# --- numpy only (scensim issue 180: apt's matplotlib and scipy break under a user-site numpy 2) --

def test_the_node_imports_neither_matplotlib_nor_scipy(tmp_path):
    """On a host whose ~/.local numpy 2.x shadows the system's 1.21, apt's matplotlib and scipy
    fail to import (243's gate on dc3f67f4); the node runs on numpy alone."""
    import subprocess
    import sys
    straight_map(tmp_path)
    code = (
        "import importlib.machinery, importlib.util, sys\n"
        f"loader = importlib.machinery.SourceFileLoader('lhc', {str(PROFILE / 'scripts' / 'lane_height_check')!r})\n"
        "m = importlib.util.module_from_spec(importlib.util.spec_from_loader('lhc', loader))\n"
        "loader.exec_module(m)\n"
        "s = m.Settings.from_vehicle({'wheel_base': 3.0, 'front_overhang': 0.8, 'rear_overhang': 1.0,"
        " 'wheel_tread': 1.6, 'left_overhang': 0.25, 'right_overhang': 0.25}, 0.138)\n"
        f"c = m.LaneHeightCheck({str(tmp_path)!r}, s)\n"
        "import numpy as np\n"
        "p = c.obstacle_points(np.array([[15.0, 1.0, 0.2]] * 5), (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0))\n"
        "m.detections(p); m.Persistence().update(m.detections(p))\n"
        "bad = sorted(k for k in sys.modules if k.split('.')[0] in ('matplotlib', 'scipy'))\n"
        "print(','.join(bad))\n")
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert out.stdout.strip() == "", out.stdout


def test_point_in_polygon_counts_edges_and_vertices_as_inside():
    """Adjacent lanelets share an edge: a point on it is on the carriageway either way."""
    sq = np.array([[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [0.0, 2.0]])
    pts = np.array([[2.0, 1.0], [5.0, 1.0], [-0.1, 1.0], [2.0, 2.1],
                    [0.0, 1.0], [4.0, 1.0], [2.0, 0.0], [2.0, 2.0],      # on the edges
                    [0.0, 0.0], [4.0, 2.0]])                             # on vertices
    assert L.inside_polygon(sq, pts).tolist() == [True, False, False, False,
                                                  True, True, True, True, True, True]


def test_point_in_polygon_on_a_concave_and_a_winding_polygon():
    # an L (concave), and the same listed clockwise: the even-odd rule is orientation-free
    ell = np.array([[0.0, 0.0], [3.0, 0.0], [3.0, 1.0], [1.0, 1.0], [1.0, 3.0], [0.0, 3.0]])
    pts = np.array([[0.5, 2.5], [2.0, 0.5], [2.0, 2.0], [0.5, 0.5], [1.0 + 1e-6, 2.0]])
    want = [True, True, False, True, False]
    assert L.inside_polygon(ell, pts).tolist() == want
    assert L.inside_polygon(ell[::-1], pts).tolist() == want
    # a ray through a vertex (y = 1, the L's reflex corner) is counted once
    assert L.inside_polygon(ell, np.array([[0.5, 1.0], [2.0, 1.0], [-1.0, 1.0]])).tolist() == \
        [True, True, False]


def test_the_disc_max_filter_matches_a_brute_force_one():
    rng = np.random.default_rng(180)
    top = np.where(rng.random((40, 30)) < 0.3, rng.normal(size=(40, 30)), -np.inf)
    got = L.disc_max(top, 3)
    want = np.full_like(top, -np.inf)
    for i in range(top.shape[0]):
        for j in range(top.shape[1]):
            for di in range(-3, 4):
                for dj in range(-3, 4):
                    if di * di + dj * dj <= 9 and 0 <= i + di < 40 and 0 <= j + dj < 30:
                        want[i, j] = max(want[i, j], top[i + di, j + dj])
    assert np.array_equal(got, want)


def test_connected_cells_are_labelled_with_8_connectivity():
    g = np.zeros((6, 6), bool)
    g[0, 0] = g[1, 1] = True               # diagonal: one group
    g[3, 3] = g[3, 4] = g[4, 4] = True     # an L: one group
    g[0, 5] = True                         # alone
    lab, n = L.label8(g)
    assert n == 3
    assert lab[0, 0] == lab[1, 1] != 0
    assert lab[3, 3] == lab[3, 4] == lab[4, 4] != lab[0, 0]
    assert lab[0, 5] not in (0, lab[0, 0], lab[3, 3])
    assert (lab[~g] == 0).all()


def test_the_lanes_raster_agrees_with_the_exact_polygon_away_from_its_edges():
    """Lanes rasterises each lanelet once (CELL_LANE cells), so a frame is a table lookup: it
    must agree with the exact test except within a cell of an edge, where it counts inside."""
    a = np.radians(np.arange(0, 181, 3))
    left = np.c_[30 * np.cos(a), 30 * np.sin(a)]
    right = np.c_[26.5 * np.cos(a), 26.5 * np.sin(a)]
    poly = np.r_[left, right[::-1]]
    lanes = L.Lanes([poly])
    rng = np.random.default_rng(180)
    pts = rng.uniform([-32, -2], [32, 32], size=(40000, 2))
    exact = L.inside_polygon(poly, pts)
    got = lanes.contains(pts)
    r = np.hypot(pts[:, 0], pts[:, 1])
    near_edge = (np.abs(r - 26.5) < 1.5 * L.CELL_LANE) | (np.abs(r - 30.0) < 1.5 * L.CELL_LANE) \
        | (np.abs(pts[:, 1]) < 1.5 * L.CELL_LANE)
    assert (got[~near_edge] == exact[~near_edge]).all()
    assert (got | ~exact).all()                     # everything inside stays inside
    on = np.c_[28.25 * np.cos(a[1:-1]), 28.25 * np.sin(a[1:-1])]
    assert lanes.contains(on).all()
    assert lanes.contains(left[5:10]).all() and lanes.contains(right[5:10]).all()
