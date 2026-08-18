import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "evaluate_trajectory.py"
SPEC = importlib.util.spec_from_file_location("evaluate_trajectory", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class EvaluateTrajectoryTest(unittest.TestCase):
    def write_csv(self, path: Path, rows):
        path.write_text("timestamp,x,y,yaw\n" + "\n".join(rows) + "\n", encoding="utf-8")

    def test_align_by_timestamp_uses_nearest_samples(self):
        est_path = None
        gt_path = None
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            est_path = temp_path / "est.csv"
            gt_path = temp_path / "gt.csv"
            self.write_csv(est_path, ["0.10,1.0,0.0,0.0", "1.60,3.0,0.0,0.0"])
            self.write_csv(gt_path, ["0.00,0.0,0.0,0.0", "1.00,2.0,0.0,0.0", "2.00,4.0,0.0,0.0"])

            est = MODULE.load_csv(est_path)
            gt = MODULE.load_csv(gt_path)
            merged = MODULE.align_by_timestamp(est, gt)
            metrics = MODULE.compute_metrics(merged)

        self.assertEqual(metrics["num_samples"], 2)
        self.assertAlmostEqual(metrics["ate_rmse"], 1.0, places=9)
        self.assertAlmostEqual(metrics["final_drift"], 1.0, places=9)

    def test_load_csv_rejects_empty_valid_samples(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "bad.csv"
            self.write_csv(csv_path, ["not_a_time,1.0,2.0,3.0"])

            with self.assertRaisesRegex(ValueError, "no valid trajectory samples"):
                MODULE.load_csv(csv_path)

    def test_load_csv_rejects_missing_required_column(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            csv_path = Path(temp_dir) / "bad.csv"
            csv_path.write_text("timestamp,x,y\n0.0,1.0,2.0\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "missing 'yaw' column"):
                MODULE.load_csv(csv_path)

    def test_main_writes_metrics_json(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = Path(temp_dir)
            est_path = temp_path / "est.csv"
            gt_path = temp_path / "gt.csv"
            out_path = temp_path / "metrics.json"
            self.write_csv(est_path, ["0.0,0.0,0.0,0.0", "1.0,1.0,0.0,0.1"])
            self.write_csv(gt_path, ["0.0,0.0,0.0,0.0", "1.0,1.0,0.0,0.1"])

            argv_before = list(__import__("sys").argv)
            try:
                __import__("sys").argv = [
                    "evaluate_trajectory.py",
                    "--est",
                    str(est_path),
                    "--gt",
                    str(gt_path),
                    "--out-json",
                    str(out_path),
                ]
                MODULE.main()
            finally:
                __import__("sys").argv = argv_before

            payload = json.loads(out_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["num_samples"], 2)
            self.assertAlmostEqual(payload["ate_rmse"], 0.0, places=12)
            self.assertAlmostEqual(payload["yaw_rmse"], 0.0, places=12)


if __name__ == "__main__":
    unittest.main()
