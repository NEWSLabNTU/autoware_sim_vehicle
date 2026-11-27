"""Every car's Autoware packages are what its spec gives (autoware_gen)."""
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent


def test_the_generated_packages_are_current():
    out = subprocess.run([sys.executable, "-m", "autoware_gen", "--check"], cwd=ROOT,
                         capture_output=True, text=True)
    assert out.returncode == 0, out.stderr


def test_the_model3_is_the_measured_car():
    info = (ROOT / "cars/tesla_model3/tesla_model3_description/config/"
            "vehicle_info.param.yaml").read_text()
    for line in ("wheel_base: 3.005", "front_overhang: 0.807", "rear_overhang: 0.98",
                 "wheel_tread: 1.667", "vehicle_height: 1.488"):
        assert line in info, line


def test_the_carla_1_0_body_is_its_own_vehicle_model():
    # Autoware reads <vehicle_model>_description's vehicle_info in places no argument reaches
    d = ROOT / "cars/tesla_model3/tesla_model3_carla_1_0_description"
    info = (d / "config/vehicle_info.param.yaml").read_text()
    for line in ("wheel_base: 2.876", "wheel_radius: 0.336", "wheel_tread: 1.58",
                 "front_overhang: 0.841", "vehicle_height: 1.443"):
        assert line in info, line
    ident = (d / "config/spec_hash.yaml").read_text()
    assert "name: tesla_model3\n" in ident and "body: carla-1.0\n" in ident
    assert "<name>tesla_model3_carla_1_0_description</name>" in (d / "package.xml").read_text()
    assert (ROOT / "cars/tesla_model3/tesla_model3_carla_1_0_launch/package.xml").is_file()
    # one sensor kit for the car, on every body
    assert not list((ROOT / "cars/tesla_model3").glob("*carla_1_0_sensor_kit*"))


def test_provisional_maps_claim_no_physics_hash(tmp_path):
    import shutil
    sys.path.insert(0, str(ROOT))
    from autoware_gen.__main__ import pedal_maps_problem, specmod
    spec = specmod.effective(specmod.load(
        ROOT / "deps/sim_vehicle_spec/vehicles/tesla_model3.yaml"), "carla-1.0")
    car = "tesla_model3_carla_1_0"
    src = ROOT / f"cars/tesla_model3/{car}_description/config"
    dst = tmp_path / f"cars/tesla_model3/{car}_description/config"
    shutil.copytree(src, dst)
    assert pedal_maps_problem(spec, tmp_path, car, default=False) is None  # measured
    (dst / "pedal_maps.yaml").write_text(
        "measured_on: provisional\ncopied_from: carla-0.9.16\nreason: not measured yet\n")
    assert pedal_maps_problem(spec, tmp_path, car, default=False) is None
    assert "default body" in pedal_maps_problem(spec, tmp_path, car, default=True)
    meta = dst / "pedal_maps.yaml"
    meta.write_text(meta.read_text() + "physics_hash: x\n")
    assert "claims no physics_hash" in pedal_maps_problem(spec, tmp_path, car, default=False)


def test_the_robot_description_survives_ros2_launch():
    # `ros2 launch` (Humble) tries every parameter value as YAML first; xacro keeps the
    # comments inside the robot, and one ": " in them makes that parse throw, so the ego
    # launch dies with "Failed to convert '<?xml ...' using yaml rules" (roadmap 070).
    # play_launch does not try, so only a plain `ros2 launch` showed it.
    files = sorted((ROOT / "cars").glob("*/*/urdf/*.xacro"))
    files += sorted((ROOT / "autoware_gen/templates").glob("*/urdf/*.xacro"))
    assert files
    for f in files:
        text = f.read_text()
        body = text[text.index("<robot"):]
        for c in re.findall(r"<!--(.*?)-->", body, re.S):
            assert ": " not in c, f"{f.relative_to(ROOT)}: comment has ': ': {c.strip()[:60]}"


def _specs(tmp_path, **blind):
    import yaml
    specs = ROOT / "deps" / "sim_vehicle_spec" / "vehicles"
    doc = yaml.safe_load((specs / "tesla_model3.yaml").read_text())
    doc["name"] = "blind_car"
    doc.pop("sensors", None)
    doc.update(blind)
    d = tmp_path / "vehicles"
    d.mkdir()
    (d / "tesla_model3.yaml").write_text((specs / "tesla_model3.yaml").read_text())
    (d / "blind_car.yaml").write_text(yaml.safe_dump(doc))
    return d


def test_a_car_that_says_autoware_car_false_gets_no_packages(tmp_path):
    """scensim 094 P2's MuSHR: no packages, no pedal maps asked for, reported skipped."""
    sys.path.insert(0, str(ROOT))
    from autoware_gen.__main__ import outputs, specs
    d = _specs(tmp_path, autoware_car=False)
    out = outputs(d, tmp_path)
    assert not any("blind_car" in p for p in out)
    assert any(p.startswith("cars/tesla_model3/") for p in out)
    skipped = []
    specs(d, skipped)
    assert skipped == ["blind_car (blind_car.yaml: autoware_car: false)"]


def test_an_autoware_car_without_sensors_is_refused(tmp_path):
    """No silent loss: a spec that wants Autoware (no autoware_car: false) and has no sensors
    is refused, naming both."""
    sys.path.insert(0, str(ROOT))
    import pytest
    from autoware_gen.__main__ import NotAnAutowareCar, outputs
    with pytest.raises(NotAnAutowareCar, match="is an Autoware car .* and has no sensors"):
        outputs(_specs(tmp_path), tmp_path)


def test_every_kit_reads_the_receivers_ins_heading():
    # scensim issue 195 L1b: with use_gnss_ins_orientation false gnss_poser takes its yaw from
    # the bearing between two fixes, random for a car standing still, and moves the antenna's
    # 0.8 m lever arm to base_link along it
    kits = sorted(ROOT.glob("cars/*/*_sensor_kit_launch/launch/gnss.launch.xml"))
    assert kits
    for k in [ROOT / "autoware_gen/templates/@KIT@_launch/launch/gnss.launch.xml", *kits]:
        text = k.read_text()
        assert '<param name="use_gnss_ins_orientation" value="true"/>' in text, k
        assert '<remap from="autoware_orientation" to="/autoware_orientation"/>' in text, k
        assert 'value="false"' not in text, k
