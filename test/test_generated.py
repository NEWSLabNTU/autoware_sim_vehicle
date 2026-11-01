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
