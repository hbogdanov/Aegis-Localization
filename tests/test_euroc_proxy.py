import csv
import tempfile
import unittest
from pathlib import Path
import zipfile

import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_bench.euroc import build_planar_proxy_samples, write_planar_proxy_csv


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    import math

    half = yaw * 0.5
    return math.cos(half), 0.0, 0.0, math.sin(half)


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    import math

    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


class EurocProxyTest(unittest.TestCase):
    def create_fixture(self, root: Path) -> None:
        imu_dir = root / "mav0" / "imu0"
        gt_dir = root / "mav0" / "state_groundtruth_estimate0"
        imu_dir.mkdir(parents=True)
        gt_dir.mkdir(parents=True)

        imu_rows = [
            ["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]"],
            ["0", "0.0", "0.0", "0.10"],
            ["1000000000", "0.0", "0.0", "0.20"],
            ["2000000000", "0.0", "0.0", "0.30"],
        ]
        gt_rows = [
            ["#timestamp [ns]", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]", "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"],
            ["0", "0.0", "0.0", "0.0", *map(str, yaw_to_quaternion(0.0))],
            ["1000000000", "1.0", "0.0", "0.0", *map(str, yaw_to_quaternion(0.1))],
            ["2000000000", "2.0", "0.5", "0.0", *map(str, yaw_to_quaternion(0.2))],
        ]

        with (imu_dir / "data.csv").open("w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerows(imu_rows)
        with (gt_dir / "data.csv").open("w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerows(gt_rows)

    def test_build_planar_proxy_samples_derives_velocities_and_yaw(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            self.create_fixture(root)

            samples = build_planar_proxy_samples(root)

            self.assertEqual(len(samples), 3)
            self.assertAlmostEqual(samples[0].vx, 1.0, places=6)
            self.assertAlmostEqual(samples[1].vy, 0.5, places=6)
            self.assertAlmostEqual(samples[0].imu_omega_z, 0.10, places=6)
            self.assertAlmostEqual(samples[2].imu_omega_z, 0.30, places=6)

    def test_write_planar_proxy_csv_outputs_canonical_columns(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "sequence"
            self.create_fixture(root)
            out_path = Path(temp_dir) / "proxy.csv"

            write_planar_proxy_csv(root, out_path)

            with out_path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                self.assertEqual(
                    reader.fieldnames,
                    ["timestamp", "x", "y", "yaw", "vx", "vy", "omega", "imu_omega_z"],
                )
                rows = list(reader)

            self.assertEqual(len(rows), 3)

    def test_build_planar_proxy_samples_supports_zip_input(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir) / "sequence"
            self.create_fixture(root)
            zip_path = Path(temp_dir) / "MH_01_easy.zip"
            with zipfile.ZipFile(zip_path, "w") as archive:
                for file_path in root.rglob("*"):
                    if file_path.is_file():
                        archive.write(file_path, file_path.relative_to(root).as_posix())

            samples = build_planar_proxy_samples(zip_path)

            self.assertEqual(len(samples), 3)
            self.assertAlmostEqual(samples[0].vx, 1.0, places=6)
            self.assertAlmostEqual(samples[2].imu_omega_z, 0.30, places=6)

    def test_planar_yaw_extraction_preserves_yaw_with_roll_and_pitch(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            imu_dir = root / "mav0" / "imu0"
            gt_dir = root / "mav0" / "state_groundtruth_estimate0"
            imu_dir.mkdir(parents=True)
            gt_dir.mkdir(parents=True)

            imu_rows = [
                ["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]", "w_RS_S_z [rad s^-1]"],
                ["0", "0.0", "0.0", "0.05"],
                ["1000000000", "0.0", "0.0", "0.05"],
            ]
            q0 = rpy_to_quaternion(0.2, -0.15, 0.4)
            q1 = rpy_to_quaternion(-0.1, 0.1, 0.6)
            gt_rows = [
                ["#timestamp [ns]", "p_RS_R_x [m]", "p_RS_R_y [m]", "p_RS_R_z [m]", "q_RS_w []", "q_RS_x []", "q_RS_y []", "q_RS_z []"],
                ["0", "0.0", "0.0", "0.0", *map(str, q0)],
                ["1000000000", "1.0", "0.0", "0.5", *map(str, q1)],
            ]

            with (imu_dir / "data.csv").open("w", encoding="utf-8", newline="") as handle:
                csv.writer(handle).writerows(imu_rows)
            with (gt_dir / "data.csv").open("w", encoding="utf-8", newline="") as handle:
                csv.writer(handle).writerows(gt_rows)

            samples = build_planar_proxy_samples(root)

            self.assertAlmostEqual(samples[0].yaw, 0.4, places=6)
            self.assertAlmostEqual(samples[1].yaw, 0.6, places=6)
            self.assertAlmostEqual(samples[0].omega, 0.2, places=6)


if __name__ == "__main__":
    unittest.main()
