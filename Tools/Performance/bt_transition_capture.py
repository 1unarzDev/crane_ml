#!/usr/bin/env python3
"""Bounded BehaviorTree transition capture with conservative completeness semantics.

This module is ROS-independent so transition/invocation behavior can be regression tested without
a ROS installation.  A BehaviorTreeLog node UID identifies a node instance in the tree; it is not
an attempt ID.  Invocation IDs here are assigned only when a configured recovery leaf transitions
from IDLE to RUNNING.
"""

from __future__ import annotations

from collections import deque
from typing import Iterable, Mapping


TERMINAL_STATUSES = frozenset(("SUCCESS", "FAILURE"))


class BehaviorTreeTransitionCapture:
    """Retain an ordered, bounded transition stream and derived recovery invocations."""

    def __init__(
        self,
        *,
        max_transitions: int,
        max_invocations: int,
        recovery_node_names: Iterable[str],
        terminal_node_names: Iterable[str],
    ) -> None:
        if max_transitions < 1 or max_invocations < 1:
            raise ValueError("BehaviorTree capture bounds must be positive")
        self.max_transitions = max_transitions
        self.max_invocations = max_invocations
        self.recovery_node_names = frozenset(recovery_node_names)
        self.terminal_node_names = frozenset(terminal_node_names)
        self.records = []
        self.recovery_invocations = []
        self._active_recovery_invocations = {}
        self._seen_fingerprints = set()
        self._fingerprint_order = deque()
        self.log_message_count = 0
        self.unique_transition_count = 0
        self.duplicate_transition_count = 0
        self.dropped_transition_count = 0
        self.recovery_invocation_count = 0
        self.dropped_recovery_invocation_count = 0
        self.goal_sent = False
        self.goal_accepted = False
        self.goal_id = None
        self.goal_accepted_after_transition_index = None
        self.terminal_transition_ids = []

    def mark_goal_sent(self) -> None:
        self.goal_sent = True

    def mark_goal_accepted(self, goal_id: str) -> None:
        self.goal_accepted = True
        self.goal_id = goal_id
        self.goal_accepted_after_transition_index = self.unique_transition_count

    def record_message(self, message_stamp: Mapping[str, int], events: Iterable[Mapping]) -> None:
        self.log_message_count += 1
        for event in events:
            self._record_transition(message_stamp, event)

    def _record_transition(self, message_stamp: Mapping[str, int], event: Mapping) -> None:
        event_stamp = event["eventStamp"]
        fingerprint = (
            int(event["uid"]),
            str(event["nodeName"]),
            str(event["previousStatus"]),
            str(event["currentStatus"]),
            int(event_stamp["sec"]),
            int(event_stamp["nanosec"]),
        )
        if fingerprint in self._seen_fingerprints:
            self.duplicate_transition_count += 1
            return
        if len(self._fingerprint_order) >= self.max_transitions:
            self._seen_fingerprints.remove(self._fingerprint_order.popleft())
        self._fingerprint_order.append(fingerprint)
        self._seen_fingerprints.add(fingerprint)
        self.unique_transition_count += 1
        record_id = f"bt-transition-{self.unique_transition_count:06d}"
        record = {
            "recordId": record_id,
            "nodeName": str(event["nodeName"]),
            "uid": int(event["uid"]),
            "previousStatus": str(event["previousStatus"]),
            "currentStatus": str(event["currentStatus"]),
            "eventStamp": dict(event_stamp),
            "messageStamp": dict(message_stamp),
            "goalId": self.goal_id,
            "goalRelation": "after_accepted_goal" if self.goal_accepted else "before_accepted_goal",
        }
        if len(self.records) < self.max_transitions:
            self.records.append(record)
        else:
            self.dropped_transition_count += 1

        self._update_recovery_invocations(record)
        if (
            self.goal_accepted
            and record["nodeName"] in self.terminal_node_names
            and record["currentStatus"] in TERMINAL_STATUSES
        ):
            self.terminal_transition_ids.append(record_id)

    def _update_recovery_invocations(self, record: Mapping) -> None:
        if not self.goal_accepted or record["nodeName"] not in self.recovery_node_names:
            return
        key = (record["uid"], record["nodeName"])
        if record["previousStatus"] == "IDLE" and record["currentStatus"] == "RUNNING":
            # A repeated publication is removed by the fingerprint check. An actual second start
            # while the first invocation is open is anomalous and must not silently become a count.
            if key in self._active_recovery_invocations:
                self._active_recovery_invocations[key]["overlappingStartTransitionId"] = record[
                    "recordId"
                ]
                return
            self.recovery_invocation_count += 1
            invocation = {
                "invocationId": f"bt-recovery-invocation-{self.recovery_invocation_count:06d}",
                "nodeName": record["nodeName"],
                "uid": record["uid"],
                "goalId": record["goalId"],
                "startTransitionId": record["recordId"],
                "endTransitionId": None,
                "terminalStatus": None,
                "complete": False,
            }
            if len(self.recovery_invocations) < self.max_invocations:
                self.recovery_invocations.append(invocation)
                self._active_recovery_invocations[key] = invocation
            else:
                self.dropped_recovery_invocation_count += 1
            return

        invocation = self._active_recovery_invocations.get(key)
        if invocation is None or record["previousStatus"] != "RUNNING":
            return
        invocation["endTransitionId"] = record["recordId"]
        invocation["terminalStatus"] = record["currentStatus"]
        invocation["complete"] = record["currentStatus"] in TERMINAL_STATUSES
        del self._active_recovery_invocations[key]

    def summary(self) -> dict:
        open_ids = sorted(
            invocation["invocationId"]
            for invocation in self._active_recovery_invocations.values()
        )
        incomplete_ids = sorted(
            invocation["invocationId"]
            for invocation in self.recovery_invocations
            if not invocation["complete"]
        )
        overlapping_ids = sorted(
            invocation["invocationId"]
            for invocation in self.recovery_invocations
            if "overlappingStartTransitionId" in invocation
        )
        reasons = [
            "BehaviorTreeLog has no publisher sequence number, so subscriber-side message loss "
            "cannot be excluded"
        ]
        pre_goal_transition_count = sum(
            record["goalRelation"] == "before_accepted_goal" for record in self.records
        )
        if not self.goal_accepted:
            reasons.append("accepted goal boundary was not observed")
        if pre_goal_transition_count:
            reasons.append(
                "transitions preceded the accepted-goal callback and were not assigned to the goal"
            )
        if not self.terminal_transition_ids:
            reasons.append("configured terminal BT transition was not observed after goal acceptance")
        if self.dropped_transition_count or self.dropped_recovery_invocation_count:
            reasons.append("bounded capture dropped records")
        if open_ids:
            reasons.append("one or more recovery invocations remained open")
        if incomplete_ids:
            reasons.append("one or more retained recovery invocations lack a terminal status")
        if overlapping_ids:
            reasons.append("one or more recovery nodes started while a prior invocation was open")
        return {
            "orderedTransitions": self.records,
            "uniqueTransitionCount": self.unique_transition_count,
            "duplicateTransitionCount": self.duplicate_transition_count,
            "deduplicationWindowTransitions": self.max_transitions,
            "droppedTransitionCount": self.dropped_transition_count,
            "recoveryInvocations": self.recovery_invocations,
            "observedRecoveryInvocationStartCount": self.recovery_invocation_count,
            "droppedRecoveryInvocationCount": self.dropped_recovery_invocation_count,
            "recoveryNodeClassifier": {
                "basis": "configured_exact_node_name_allowlist",
                "nodeNames": sorted(self.recovery_node_names),
            },
            "completeness": {
                "historyStatus": "not_proven",
                "observerCreatedBeforeGoalSend": True,
                "goalSent": self.goal_sent,
                "goalAccepted": self.goal_accepted,
                "goalAcceptedAfterTransitionIndex": self.goal_accepted_after_transition_index,
                "retainedPreAcceptedGoalTransitionCount": pre_goal_transition_count,
                "configuredTerminalNodeNames": sorted(self.terminal_node_names),
                "terminalTransitionObserved": bool(self.terminal_transition_ids),
                "terminalTransitionIds": self.terminal_transition_ids,
                "openRecoveryInvocationIds": open_ids,
                "incompleteRecoveryInvocationIds": incomplete_ids,
                "overlappingRecoveryInvocationIds": overlapping_ids,
                "messageLossDetectable": False,
                "exactRecoveryCountEligible": False,
                "limitations": reasons,
            },
        }
