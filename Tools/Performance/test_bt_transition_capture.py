#!/usr/bin/env python3
"""Unit tests for bounded BehaviorTree transition and invocation capture."""

import sys
from pathlib import Path
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).resolve().parent))

from bt_transition_capture import (  # noqa: E402
    BehaviorTreeTransitionCapture,
    direct_terminal_recovery_nodes_from_bt_xml,
)


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
                transition(7, "Wait", "SUCCESS", "IDLE", 3),
                transition(7, "Wait", "IDLE", "RUNNING", 4),
                transition(7, "Wait", "RUNNING", "FAILURE", 5),
            ],
        )

        invocations = capture.summary()["recoveryInvocations"]
        self.assertEqual(
            [item["invocationId"] for item in invocations],
            ["bt-recovery-invocation-000001", "bt-recovery-invocation-000002"],
        )
        self.assertEqual([item["terminalStatus"] for item in invocations], ["SUCCESS", "FAILURE"])
        self.assertTrue(all(item["complete"] for item in invocations))
        self.assertTrue(all(
            item["observationPattern"] == "idle_to_running" for item in invocations
        ))

    def test_idle_to_success_is_one_complete_single_tick_invocation(self):
        capture = self.make_capture(
            recovery_node_names=("ClearLocalCostmap",),
            direct_terminal_recovery_node_names=("ClearLocalCostmap",),
        )
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        direct = transition(9, "ClearLocalCostmap", "IDLE", "SUCCESS", 1)
        capture.record_message({"sec": 10, "nanosec": 2}, [direct])
        capture.record_message({"sec": 10, "nanosec": 3}, [direct])

        summary = capture.summary()
        self.assertEqual(summary["observedRecoveryInvocationStartCount"], 1)
        self.assertEqual(summary["duplicateTransitionCount"], 1)
        invocation = summary["recoveryInvocations"][0]
        self.assertEqual(invocation["startTransitionId"], "bt-transition-000001")
        self.assertEqual(invocation["endTransitionId"], "bt-transition-000001")
        self.assertEqual(invocation["terminalStatus"], "SUCCESS")
        self.assertEqual(invocation["observationPattern"], "idle_to_terminal")
        self.assertTrue(invocation["complete"])
        self.assertEqual(summary["completeness"]["openRecoveryInvocationIds"], [])

    def test_idle_to_failure_is_not_admitted_as_a_single_tick_invocation(self):
        capture = self.make_capture(
            recovery_node_names=("ClearGlobalCostmap",),
            direct_terminal_recovery_node_names=("ClearGlobalCostmap",),
        )
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 2},
            [transition(10, "ClearGlobalCostmap", "IDLE", "FAILURE", 1)],
        )

        self.assertEqual(capture.summary()["recoveryInvocations"], [])

    def test_reset_delimits_two_single_tick_invocations(self):
        capture = self.make_capture(
            recovery_node_names=("ClearLocalCostmap",),
            direct_terminal_recovery_node_names=("ClearLocalCostmap",),
        )
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [
                transition(9, "ClearLocalCostmap", "IDLE", "SUCCESS", 1),
                transition(9, "ClearLocalCostmap", "SUCCESS", "IDLE", 2),
                transition(9, "ClearLocalCostmap", "IDLE", "SUCCESS", 3),
            ],
        )

        self.assertEqual(len(capture.summary()["recoveryInvocations"]), 2)

    def test_restart_without_observed_reset_is_not_counted(self):
        capture = self.make_capture(
            recovery_node_names=("ClearLocalCostmap",),
            direct_terminal_recovery_node_names=("ClearLocalCostmap",),
        )
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 9},
            [
                transition(9, "ClearLocalCostmap", "IDLE", "SUCCESS", 1),
                transition(9, "ClearLocalCostmap", "IDLE", "SUCCESS", 2),
            ],
        )

        summary = capture.summary()
        self.assertEqual(len(summary["recoveryInvocations"]), 1)
        self.assertEqual(
            summary["completeness"]["restartWithoutResetRecoveryInvocationIds"],
            ["bt-recovery-invocation-000001"],
        )
        self.assertIn(
            "restartWithoutResetTransitionId", summary["recoveryInvocations"][0]
        )

    def test_unconfigured_idle_to_terminal_leaf_is_not_a_recovery_invocation(self):
        capture = self.make_capture()
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 2},
            [transition(11, "WouldAControllerRecoveryHelp", "IDLE", "SUCCESS", 1)],
        )

        summary = capture.summary()
        self.assertEqual(summary["observedRecoveryInvocationStartCount"], 0)
        self.assertEqual(summary["recoveryInvocations"], [])

    def test_direct_terminal_edge_requires_explicit_node_classification(self):
        capture = self.make_capture(recovery_node_names=("Wait",))
        capture.mark_goal_sent()
        capture.mark_goal_accepted("goal-1")
        capture.record_message(
            {"sec": 10, "nanosec": 2},
            [transition(7, "Wait", "IDLE", "SUCCESS", 1)],
        )

        self.assertEqual(capture.summary()["recoveryInvocations"], [])

    def test_direct_terminal_nodes_must_be_recovery_nodes(self):
        with self.assertRaisesRegex(ValueError, "must also be configured recovery nodes"):
            self.make_capture(direct_terminal_recovery_node_names=("ClearLocalCostmap",))

    def test_loaded_tree_classifies_only_unconditional_clear_costmap_leaves(self):
        xml = """<root><BehaviorTree ID="MainTree">
          <Sequence>
            <ClearEntireCostmap name="ClearLocalCostmap-Context"/>
            <ClearEntireCostmap name="ConditionalClear" _skipIf="{skip}"/>
            <WouldAControllerRecoveryHelp name="Guard"/>
          </Sequence>
        </BehaviorTree></root>"""
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tree.xml"
            path.write_text(xml, encoding="utf-8")
            self.assertEqual(
                direct_terminal_recovery_nodes_from_bt_xml(path),
                ("ClearLocalCostmap-Context",),
            )

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
                transition(7, "Wait", "SUCCESS", "IDLE", 3),
                transition(7, "Wait", "IDLE", "RUNNING", 4),
            ],
        )

        summary = capture.summary()
        self.assertEqual(len(summary["orderedTransitions"]), 1)
        self.assertEqual(summary["transitionCapacity"], 1)
        self.assertEqual(summary["retainedTransitionCount"], 1)
        self.assertEqual(summary["droppedTransitionCount"], 3)
        self.assertEqual(len(summary["recoveryInvocations"]), 1)
        self.assertEqual(summary["recoveryInvocationCapacity"], 1)
        self.assertEqual(summary["retainedRecoveryInvocationCount"], 1)
        self.assertEqual(summary["droppedRecoveryInvocationCount"], 1)
        self.assertIn("bounded capture dropped records", summary["completeness"]["limitations"])


if __name__ == "__main__":
    unittest.main()
