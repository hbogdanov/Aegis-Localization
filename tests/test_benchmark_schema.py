import json
import tempfile
import unittest
from pathlib import Path

import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
PYTHON_ROOT = REPO_ROOT / "python"
if str(PYTHON_ROOT) not in sys.path:
    sys.path.insert(0, str(PYTHON_ROOT))

from aegis_bench import create_run_layout
from aegis_eval import write_json


class BenchmarkSchemaTest(unittest.TestCase):
    def test_create_run_layout_builds_canonical_directories(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            layout = create_run_layout(temp_dir, "synthetic", "low_noise", "run_001")

            self.assertTrue(layout.root.exists())
            self.assertTrue(layout.normalized_dir.exists())
            self.assertTrue(layout.metrics_dir.exists())
            self.assertTrue(layout.plots_dir.exists())
            self.assertTrue(layout.logs_dir.exists())
            self.assertEqual(layout.ground_truth_csv.name, "ground_truth.csv")
            self.assertEqual(layout.estimator_csv("ekf").name, "ekf.csv")

    def test_write_json_creates_parent_directories(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            out_path = Path(temp_dir) / "nested" / "metadata.json"
            write_json(out_path, {"backend": "synthetic"})

            self.assertEqual(json.loads(out_path.read_text(encoding="utf-8"))["backend"], "synthetic")


if __name__ == "__main__":
    unittest.main()
