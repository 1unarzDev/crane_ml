import json
import os
import shutil
import subprocess
import tempfile
import unittest
import zipfile
from pathlib import Path
from unittest import mock

from crane_cluster import config, miniyaml, provision
from crane_cluster.gitcache import GitCache

HERE = Path(__file__).resolve().parents[1]


class Provision(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.d = Path(self.tmp.name)
        self.cfg = self.d / "conf" / "worker.yaml"
        self.cfg.parent.mkdir()
        self.cfg.write_text(f"coordinator: http://x\ndata_dir: {self.d / 'data'}\nbackends:\n  dummy: {{type: dummy}}\n"
                            "caps:\n  tags: [lab]\n", encoding="utf-8-sig")

    def tearDown(self):
        self.tmp.cleanup()

    def test_shipped_plan_parses_and_names_known_tools(self):
        plan = miniyaml.load_file(HERE / "install" / "provision.yaml")
        for name in plan["tools"]:
            self.assertTrue(hasattr(provision.Provisioner, f"tool_{name}"), name)
        self.assertEqual(plan["backends"]["fastsim_roboboat"]["requires"], ["repo:mhseals_testing", "dotnet", "unity_managed"])

    def test_backends_are_added_only_when_missing_and_their_requirements_are_met(self):
        plan = {"backends": {
            "dummy": {"requires": [], "config": {"type": "dummy"}},
            "fast": {"requires": ["repo:a", "dotnet"], "config": {"type": "pkg.mod:B", "path": "{repos}/a",
                                                                 "python": "{python}", "workers": 4, "repos": {"a": "https://h/a.git"}}},
            "gpu": {"requires": ["player"], "config": {"type": "u", "player": "{player}"}}}}
        pv = provision.Provisioner(plan, self.d / "tools", self.cfg)
        pv.have = {"repo:a", "dotnet"}
        pv.backends()
        got = config.load(self.cfg)
        self.assertEqual(set(got["backends"]), {"dummy", "fast"})
        self.assertEqual(got["backends"]["fast"]["path"], (self.d / "data" / "repos" / "a").as_posix())
        self.assertEqual(got["backends"]["fast"]["workers"], 4)
        self.assertEqual(got["backends"]["fast"]["repos"], {"a": "https://h/a.git"})
        self.assertEqual(got["caps"]["tags"], ["lab"])
        before = self.cfg.read_text()
        provision.Provisioner(plan, self.d / "tools", self.cfg).backends()  # a second run keeps the file
        self.assertEqual(self.cfg.read_text(), before)

    def test_tools_json_goes_on_path_and_env(self):
        (self.cfg.parent / "tools.json").write_text(json.dumps({"path": [str(self.d / "bin")], "env": {"UnityManaged": "/u"}}))
        with mock.patch.dict(os.environ, {"PATH": "/usr/bin"}):
            provision.apply_tools_env(self.cfg)
            provision.apply_tools_env(self.cfg)
            self.assertEqual(os.environ["PATH"].split(os.pathsep), [str(self.d / "bin"), "/usr/bin"])
            self.assertEqual(os.environ["UnityManaged"], "/u")

    def test_archive_install_flattens_and_skips_the_same_asset(self):
        z = self.d / "tool-1.0.zip"
        with zipfile.ZipFile(z, "w") as f:
            f.writestr("tool-1.0/bin/tool", "x")
        dest = self.d / "tools" / "tool"
        self.assertTrue(provision.install_archive(z.name, z.as_uri(), dest))
        self.assertTrue((dest / "bin" / "tool").is_file())
        self.assertFalse(provision.install_archive(z.name, z.as_uri(), dest))

    def test_caps_tags_merge_configured_and_found(self):
        with mock.patch.object(config, "_gpu", return_value=(None, 0, 0.0)), \
             mock.patch.object(config.shutil, "which", side_effect=lambda t: "/x" if t == "git" else None):
            caps = config.detect_caps({"caps": {"tags": ["lab"], "vulkan": True}}, ["dummy"])
        self.assertEqual(caps["tags"], ["lab", caps["os"], "vulkan", "git"])

    @unittest.skipUnless(shutil.which("git"), "git")
    def test_sparse_checkout_through_the_cache(self):
        src = self.d / "src"
        src.mkdir()
        run = lambda *a: subprocess.run(["git", *a], cwd=src, check=True, capture_output=True)
        run("init", "-q", "-b", "main")
        for f in ("keep/a.txt", "drop/b.txt"):
            (src / f).parent.mkdir(exist_ok=True)
            (src / f).write_text(f)
        run("add", ".")
        run("-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x")
        dest = self.d / "co"
        GitCache(self.d / "git").checkout("src", str(src), "main", dest, sparse=["keep"])
        self.assertTrue((dest / "keep" / "a.txt").is_file())
        self.assertFalse((dest / "drop").exists())


if __name__ == "__main__":
    unittest.main()
