#!/usr/bin/env python3
"""Unit tests for bounded BehaviorTree transition and invocation capture."""

import sys
from pathlib import Path
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from bt_transition_capture import BehaviorTreeTransitionCapture  # noqa: E402


def transition(uid, node, previous, current, nanosec):
    return {
        "uid": uid,
        "nodeName": node,
        "previousStatus": previous,
        "currentStatus": current,
        "eventStamp": {"sec": 10, "nanosec": nanosec},
    }


class BehaviorTreeTransitionCaptureTests(unittest.TestCase):
    def make_capture(self, **overrides):
        options = {
            "max_transitions": 10,
            "max_invocations": 10,
            "recovery_node_names": ("Wait",),
            "terminal_node_names": ("NavigateRecovery",),
        }
        options.update(overrides)
        return BehaviorTreeTransitionCapture(**options)

    def test_duplicate_publication_does_not_create_a_second_invocation(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        start = transition(7, "Wait", "IDLE", "RUNNING", 1)
        capture.record_message({"sec": 10, "nanosec": 2}, [start])
        capture.record_message({"sec": 10, "nanosec": 3}, [start])

        summary = capture.summary()
        self.assertEqual(summary["uniqueTransitionCount"], 1)
        self.assertEqual(summary["duplicateTransitionCount"], 1)
        self.assertEqual(summary["observedRecoveryInvocationStartCount"], 1)

    def test_pre_acceptance_transition_is_retained_but_not_assigned_as_an_invocation(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.record_message(
            {"sec": 10, "nanosec": 2},
            [transition(7, "Wait", "IDLE", "RUNNING", 1)],
        )
        capture.mark_goal_accepted("goal-1")

        summary = capture.summary()
        self.assertEqual(summary["observedRecoveryInvocationStartCount"], 0)
        self.assertEqual(
            summary["completeness"]["retainedPreAcceptedGoalTransitionCount"], 1
        )
        self.assertIsNone(summary["orderedTransitions"][0]["goalId"])

    def test_two_idle_to_running_edges_receive_unique_invocation_ids(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [
                transition(7, "Wait", "IDLE", "RUNNING", 1),
                transition(7, "Wait", "RUNNING", "SUCCESS", 2),
                transition(7, "Wait", "IDLE", "RUNNING", 3),
                transition(7, "Wait", "RUNNING", "FAILURE", 4),
            ],
        )

        invocations = capture.summary()["recoveryInvocations"]
        self.assertEqual(
            [item["invocationId"] for item in invocations],
            ["bt-recovery-invocation-000001", "bt-recovery-invocation-000002"],
        )
        self.assertEqual([item["terminalStatus"] for item in invocations], ["SUCCESS", "FAILURE"])
        self.assertTrue(all(item["complete"] for item in invocations))

    def test_open_invocation_and_absent_terminal_are_explicitly_incomplete(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [transition(7, "Wait", "IDLE", "RUNNING", 1)],
        )

        completeness = capture.summary()["completeness"]
        self.assertEqual(completeness["historyStatus"], "not_proven")
        self.assertFalse(completeness["terminalTransitionObserved"])
        self.assertFalse(completeness["exactRecoveryCountEligible"])
        self.assertEqual(
            completeness["openRecoveryInvocationIds"], ["bt-recovery-invocation-000001"]
        )
        self.assertEqual(
            completeness["incompleteRecoveryInvocationIds"],
            ["bt-recovery-invocation-000001"],
        )

    def test_running_to_idle_closes_but_does_not_complete_an_invocation(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [
                transition(7, "Wait", "IDLE", "RUNNING", 1),
                transition(7, "Wait", "RUNNING", "IDLE", 2),
            ],
        )

        summary = capture.summary()
        self.assertEqual(summary["completeness"]["openRecoveryInvocationIds"], [])
        self.assertEqual(
            summary["completeness"]["incompleteRecoveryInvocationIds"],
            ["bt-recovery-invocation-000001"],
        )
        self.assertFalse(summary["recoveryInvocations"][0]["complete"])

    def test_terminal_transition_is_recorded_but_does_not_prove_no_message_loss(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [transition(1, "NavigateRecovery", "RUNNING", "SUCCESS", 1)],
        )

        completeness = capture.summary()["completeness"]
        self.assertTrue(completeness["terminalTransitionObserved"])
        self.assertFalse(completeness["messageLossDetectable"])
        self.assertFalse(completeness["exactRecoveryCountEligible"])

    def test_bounds_report_dropped_records_without_unbounded_retention(self):
        capture = self.make_capture(max_transitions=1, max_invocations=1)
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [
                transition(7, "Wait", "IDLE", "RUNNING", 1),
                transition(7, "Wait", "RUNNING", "SUCCESS", 2),
                transition(7, "Wait", "IDLE", "RUNNING", 3),
            ],
        )

        summary = capture.summary()
        self.assertEqual(len(summary["orderedTransitions"]), 1)
        self.assertEqual(summary["droppedTransitionCount"], 2)
        self.assertEqual(len(summary["recoveryInvocations"]), 1)
        self.assertEqual(summary["droppedRecoveryInvocationCount"], 1)
        self.assertIn("bounded capture dropped records", summary["completeness"]["limitations"])


if __name__ == "__main__":
    unittest.main()
