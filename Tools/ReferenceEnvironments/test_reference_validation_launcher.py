import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCHER = ROOT / "Tools" / "ReferenceEnvironments" / "run_reference_validation.sh"


class ReferenceValidationLauncherTests(unittest.TestCase):
    def test_help_lists_supported_targets(self) -> None:
        result = subprocess.run(
            ["bash", str(LAUNCHER), "--help"],
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertIn("px4-walls", result.stdout)
        self.assertIn("clearpath-pipeline", result.stdout)
        self.assertIn("f1tenth-spielberg", result.stdout)
        self.assertIn("turtlebot3-warehouse", result.stdout)
        self.assertIn("land-proving-ground", result.stdout)

    def test_rejects_scene_absent_from_build_manifest(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            player = build / "CRANE.x86_64"
            player.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
            player.chmod(0o755)
            (build / "crane-build-manifest.json").write_text(
                json.dumps({"scenes": ["Assets/Scenes/PX4 Walls Validation.unity"]}),
                encoding="utf-8",
            )
            environment = os.environ | {"CRANE_PLAYER": str(player)}
            result = subprocess.run(
                ["bash", str(LAUNCHER), "f1tenth-spielberg"],
                capture_output=True,
                text=True,
                env=environment,
            )
        self.assertNotEqual(0, result.returncode)
        self.assertIn("not in", result.stderr)
        self.assertIn("--crane-extra-scene", result.stderr)

    def test_px4_run_is_graphics_free_and_disables_ros(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            player = build / "CRANE.x86_64"
            arguments = build / "arguments.txt"
            player.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\" > \"${CRANE_FAKE_ARGS}\"\n",
                encoding="utf-8",
            )
            player.chmod(0o755)
            (build / "crane-build-manifest.json").write_text(
                json.dumps({"scenes": ["Assets/Scenes/PX4 Walls Validation.unity"]}),
                encoding="utf-8",
            )
            environment = os.environ | {
                "CRANE_PLAYER": str(player),
                "CRANE_FAKE_ARGS": str(arguments),
                "CRANE_REFERENCE_RESULT_ROOT": str(build / "results"),
            }
            subprocess.run(
                ["bash", str(LAUNCHER), "px4-walls"],
                check=True,
                env=environment,
            )
            invoked = arguments.read_text(encoding="utf-8").splitlines()
        self.assertIn("-batchmode", invoked)
        self.assertIn("-nographics", invoked)
        self.assertIn("--crane-profile", invoked)
        self.assertIn("train-cpu", invoked)
        self.assertIn("--crane-scene", invoked)
        self.assertIn("--crane-disable-ros", invoked)
        self.assertIn("--crane-aerial-validation", invoked)
        self.assertIn("PX4 Walls Validation", invoked)

    def test_warehouse_run_passes_manifest_identity_to_generic_validator(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            player = build / "CRANE.x86_64"
            arguments = build / "arguments.txt"
            player.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\" > \"${CRANE_FAKE_ARGS}\"\n",
                encoding="utf-8",
            )
            player.chmod(0o755)
            (build / "crane-build-manifest.json").write_text(
                json.dumps({"scenes": ["Assets/Scenes/TurtleBot3 Warehouse Validation.unity"]}),
                encoding="utf-8",
            )
            environment = os.environ | {
                "CRANE_PLAYER": str(player),
                "CRANE_FAKE_ARGS": str(arguments),
                "CRANE_REFERENCE_RESULT_ROOT": str(build / "results"),
            }
            subprocess.run(
                ["bash", str(LAUNCHER), "turtlebot3-warehouse"],
                check=True,
                env=environment,
            )
            invoked = arguments.read_text(encoding="utf-8").splitlines()
        self.assertIn("-nographics", invoked)
        self.assertIn("--crane-reference-environment-id", invoked)
        self.assertIn("crane-industrial-warehouse-v2", invoked)
        self.assertIn("--crane-reference-manifest-sha256", invoked)

    def test_land_proving_ground_uses_manifest_hash_and_runtime_layout(self) -> None:
        text = LAUNCHER.read_text(encoding="utf-8")
        self.assertIn("land-proving-ground)", text)
        self.assertIn("crane_land_proving_ground_${catalog}.json", text)
        self.assertIn("--crane-land-proving-ground-catalog", text)
        self.assertIn("--crane-land-proving-ground-layout", text)
        self.assertIn('CRANE_PROVING_GROUND_CATALOG:-v1', text)
        self.assertIn("CRANE_PROVING_GROUND_LAYOUT:-alternate-corridors-v1", text)

    def test_land_proving_ground_v2_passes_corrected_catalog_identity(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            build = Path(directory)
            player = build / "CRANE.x86_64"
            arguments = build / "arguments.txt"
            player.write_text(
                "#!/usr/bin/env bash\nprintf '%s\\n' \"$@\" > \"${CRANE_FAKE_ARGS}\"\n",
                encoding="utf-8",
            )
            player.chmod(0o755)
            (build / "crane-build-manifest.json").write_text(
                json.dumps({"scenes": ["Assets/Scenes/TurtleBot3 Warehouse Validation.unity"]}),
                encoding="utf-8",
            )
            environment = os.environ | {
                "CRANE_PLAYER": str(player),
                "CRANE_FAKE_ARGS": str(arguments),
                "CRANE_REFERENCE_RESULT_ROOT": str(build / "results"),
                "CRANE_PROVING_GROUND_CATALOG": "v2",
                "CRANE_PROVING_GROUND_LAYOUT": "slalom-s-turn-v2",
            }
            subprocess.run(
                ["bash", str(LAUNCHER), "land-proving-ground"],
                check=True,
                env=environment,
            )
            invoked = arguments.read_text(encoding="utf-8").splitlines()
        self.assertIn("crane-land-proving-ground-v2", invoked)
        self.assertIn("2.0.0", invoked)
        self.assertIn("v2", invoked)
        self.assertIn("slalom-s-turn-v2", invoked)


if __name__ == "__main__":
    unittest.main()
