"""Meta-test: with the loophole detector disabled, the suite must go red."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
TARGET = "test_must_fail.py::test_naive_reward_trusting_optimizer_finds_loophole_and_is_flagged"


def _run(env_extra):
    env = {**os.environ, **env_extra}
    return subprocess.run([sys.executable, "-I", "-m", "pytest", "-q", "-x", "--no-header", "-p", "no:cacheprovider",
                           os.path.join(HERE, TARGET)], cwd=os.path.dirname(HERE),
                          env=env, capture_output=True, text=True, timeout=300)


def test_disabling_loophole_detector_turns_the_suite_red():
    red = _run({"CRANE_EXP_DISABLE_LOOPHOLE_DETECTOR": "1"})
    assert red.returncode != 0, "suite stayed green with the loophole detector disabled: detector is not tested"
    assert "1 failed" in red.stdout, red.stdout[-800:]
    assert "LOOPHOLE" in red.stdout or "loophole" in red.stdout


def test_same_target_is_green_with_detector_enabled():
    env = {k: v for k, v in os.environ.items() if k != "CRANE_EXP_DISABLE_LOOPHOLE_DETECTOR"}
    r = subprocess.run([sys.executable, "-I", "-m", "pytest", "-q", "-x", "--no-header", "-p", "no:cacheprovider",
                        os.path.join(HERE, TARGET)], cwd=os.path.dirname(HERE), env=env,
                       capture_output=True, text=True, timeout=300)
    assert r.returncode == 0, r.stdout[-800:]
