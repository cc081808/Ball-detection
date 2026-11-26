import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import Mock

from update_receiver import update


class UpdateTests(TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.project = self.root / "vex-receiver"
        (self.project / ".vscode").mkdir(parents=True)
        (self.project / "build").mkdir()
        self.settings = self.project / ".vscode/vex_project_settings.json"
        self.settings.write_text(json.dumps({"project": {"name": "ball-serial", "slot": 4}}))
        (self.project / "build/ball-serial.bin").write_bytes(b"SENSOR + CV V5.1\0BALL_SERIAL_V5 ")
        self.ports = [SimpleNamespace(device="COM7", description="VEX IQ Communications Port")]

    def test_wrong_slot_prevents_any_commands(self):
        self.settings.write_text(json.dumps({"project": {"name": "ball-serial", "slot": 1}}))
        run = Mock()
        with self.assertRaisesRegex(RuntimeError, "slot 4"):
            update(self.root, run, self.ports)
        run.assert_not_called()

    def test_failed_build_never_downloads_old_binary(self):
        run = Mock(return_value=SimpleNamespace(returncode=1))
        with self.assertRaisesRegex(RuntimeError, "Build failed"):
            update(self.root, run, self.ports)
        self.assertEqual(run.call_count, 1)

    def test_wrong_binary_never_downloaded(self):
        (self.project / "build/ball-serial.bin").write_bytes(b"TEST")
        run = Mock(return_value=SimpleNamespace(returncode=0))
        with self.assertRaisesRegex(RuntimeError, "confirmation"):
            update(self.root, run, self.ports)
        self.assertEqual(run.call_count, 1)

    def test_previous_trim_only_build_is_rejected(self):
        (self.project / "build/ball-serial.bin").write_bytes(b"LED: AUTO AREA\0BALL_SERIAL_V3 ")
        run = Mock(return_value=SimpleNamespace(returncode=0))
        with self.assertRaisesRegex(RuntimeError, "confirmation"):
            update(self.root, run, self.ports)
        self.assertEqual(run.call_count, 1)

    def test_camera_steering_build_is_rejected(self):
        (self.project / "build/ball-serial.bin").write_bytes(b"VISION FIRST V4\0BALL_SERIAL_V4 ")
        run = Mock(return_value=SimpleNamespace(returncode=0))
        with self.assertRaisesRegex(RuntimeError, "confirmation"):
            update(self.root, run, self.ports)
        self.assertEqual(run.call_count, 1)

    def test_success_targets_only_slot_four(self):
        run = Mock(return_value=SimpleNamespace(returncode=0))
        update(self.root, run, self.ports)
        args = run.call_args_list[1].args[0]
        self.assertEqual(args[1:3], ["--slot", "4"])
        self.assertEqual(args[-1], "COM7")
        self.assertEqual(run.call_count, 2)

    def test_previous_startup_build_is_rejected(self):
        (self.project / "build/ball-serial.bin").write_bytes(b"SENSOR + CV V5\0BALL_SERIAL_V5 ")
        run = Mock(return_value=SimpleNamespace(returncode=0))
        with self.assertRaisesRegex(RuntimeError, "confirmation"):
            update(self.root, run, self.ports)
        self.assertEqual(run.call_count, 1)
