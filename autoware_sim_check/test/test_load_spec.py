"""vehicle_check reads a spec from a path, for a car whose spec lives in another repository."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "deps/sim_vehicle_spec"))

from autoware_sim_check.vehicle_check import load_spec  # noqa: E402

MODEL3 = ROOT / "deps/sim_vehicle_spec/vehicles/tesla_model3.yaml"


def test_a_spec_path_is_read_without_the_package_index(tmp_path):
    spec = tmp_path / "my_car.yaml"
    spec.write_text(MODEL3.read_text())
    s = load_spec("not_in_sim_vehicle_spec", spec_path=str(spec), description_dir=str(tmp_path))
    assert s.doc["name"] == "tesla_model3"
    assert s.body == "carla-0.9.16"  # the default body


def test_a_body_package_names_its_body():
    # tesla_model3_carla_1_0 is tesla_model3 on its carla-1.0 body (autoware_gen's spec_hash.yaml)
    desc = ROOT / "cars/tesla_model3/tesla_model3_carla_1_0_description"
    s = load_spec("tesla_model3_carla_1_0", spec_path=str(MODEL3), description_dir=str(desc))
    assert s.body == "carla-1.0"
    assert s.doc["geometry"]["wheel_base"] == 2.876


def test_a_missing_spec_path_is_an_error(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_spec("tesla_model3", spec_path=str(tmp_path / "absent.yaml"),
                  description_dir=str(tmp_path))
