#!/usr/bin/env python3
"""Bounded BehaviorTree transition capture with conservative completeness semantics.

This module is ROS-independent so transition/invocation behavior can be regression tested without
a ROS installation.  A BehaviorTreeLog node UID identifies a node instance in the tree; it is not
an attempt ID. Invocation IDs are assigned when a configured recovery leaf leaves IDLE for RUNNING
or a source-verified service leaf completes directly at SUCCESS within one BehaviorTree tick.
"""

from __future__ import annotations

from collections import deque
from pathlib import Path
from typing import Iterable, Mapping
import xml.etree.ElementTree as ET


TERMINAL_STATUSES = frozenset(("SUCCESS", "FAILURE"))
COMPLETION_OVERRIDE_ATTRIBUTES = frozenset(("_skipIf", "_failureIf", "_successIf", "_while"))


def direct_terminal_recovery_nodes_from_bt_xml(path: Path) -> tuple[str, ...]:
    """Return source-verified ClearEntireCostmap instance names without BT preconditions."""
    root = ET.parse(path).getroot()
    names = set()
    for element in root.iter():
        tag = element.tag.rsplit("}", 1)[-1]
        if tag != "ClearEntireCostmap":
            continue
        if COMPLETION_OVERRIDE_ATTRIBUTES.intersection(element.attrib):
            continue
        name = element.attrib.get("name")
        if name:
            names.add(name)
    return tuple(sorted(names))


class BehaviorTreeTransitionCapture:
    """Retain an ordered, bounded transition stream and derived recovery invocations."""

    def __init__(
        self,
        *,
        max_transitions: int,
        max_invocations: int,
        recovery_node_names: Iterable[str],
        terminal_node_names: Iterable[str],
        direct_terminal_recovery_node_names: Iterable[str] = (),
        direct_terminal_classifier_basis: str = "explicit_configuration",
        direct_terminal_classifier_sha256: str | None = None,
    ) -> None:
        if max_transitions < 1 or max_invocations < 1:
            raise ValueError("BehaviorTree capture bounds must be positive")
        self.max_transitions = max_transitions
        self.max_invocations = max_invocations
        self.recovery_node_names = frozenset(recovery_node_names)
        self.direct_terminal_recovery_node_names = frozenset(
            direct_terminal_recovery_node_names
        )
        self.direct_terminal_classifier_basis = direct_terminal_classifier_basis
        self.direct_terminal_classifier_sha256 = direct_terminal_classifier_sha256
        unknown_direct_nodes = self.direct_terminal_recovery_node_names - self.recovery_node_names
        if unknown_direct_nodes:
            raise ValueError(
                "Direct-terminal recovery nodes must also be configured recovery nodes: "
                + ", ".join(sorted(unknown_direct_nodes))
            )
        self.terminal_node_names = frozenset(terminal_node_names)
        self.records = []
        self.recovery_invocations = []
        self._active_recovery_invocations = {}
        self._recovery_nodes_awaiting_reset = {}
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
        if (
            record["previousStatus"] in TERMINAL_STATUSES
            and record["currentStatus"] == "IDLE"
        ):
            self._recovery_nodes_awaiting_reset.pop(key, None)
            return
        leaves_idle = record["previousStatus"] == "IDLE"
        starts_running = leaves_idle and record["currentStatus"] == "RUNNING"
        completes_in_tick = (
            leaves_idle
            and record["nodeName"] in self.direct_terminal_recovery_node_names
            and record["currentStatus"] == "SUCCESS"
        )
        if starts_running or completes_in_tick:
            # A repeated publication is removed by the fingerprint check. An actual second start
            # while the first invocation is open is anomalous and must not silently become a count.
            if key in self._active_recovery_invocations:
                self._active_recovery_invocations[key]["overlappingStartTransitionId"] = record[
                    "recordId"
                ]
                return
            if key in self._recovery_nodes_awaiting_reset:
                self._recovery_nodes_awaiting_reset[key][
                    "restartWithoutResetTransitionId"
                ] = record["recordId"]
                return
            self.recovery_invocation_count += 1
            invocation = {
                "invocationId": f"bt-recovery-invocation-{self.recovery_invocation_count:06d}",
                "nodeName": record["nodeName"],
                "uid": record["uid"],
                "goalId": record["goalId"],
                "startTransitionId": record["recordId"],
                "endTransitionId": record["recordId"] if completes_in_tick else None,
                "terminalStatus": record["currentStatus"] if completes_in_tick else None,
                "complete": completes_in_tick,
                "observationPattern": (
                    "idle_to_terminal" if completes_in_tick else "idle_to_running"
                ),
            }
            if len(self.recovery_invocations) < self.max_invocations:
                self.recovery_invocations.append(invocation)
                if not completes_in_tick:
                    self._active_recovery_invocations[key] = invocation
            else:
                self.dropped_recovery_invocation_count += 1
            if completes_in_tick:
                self._recovery_nodes_awaiting_reset[key] = invocation
            return

        invocation = self._active_recovery_invocations.get(key)
        if invocation is None or record["previousStatus"] != "RUNNING":
            return
        invocation["endTransitionId"] = record["recordId"]
        invocation["terminalStatus"] = record["currentStatus"]
        invocation["complete"] = record["currentStatus"] in TERMINAL_STATUSES
        del self._active_recovery_invocations[key]
        if invocation["complete"]:
            self._recovery_nodes_awaiting_reset[key] = invocation

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
        restart_without_reset_ids = sorted(
            invocation["invocationId"]
            for invocation in self.recovery_invocations
            if "restartWithoutResetTransitionId" in invocation
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
        if restart_without_reset_ids:
            reasons.append(
                "one or more recovery nodes appeared to restart without an observed reset"
            )
        return {
            "orderedTransitions": self.records,
            "transitionCapacity": self.max_transitions,
            "retainedTransitionCount": len(self.records),
            "uniqueTransitionCount": self.unique_transition_count,
            "duplicateTransitionCount": self.duplicate_transition_count,
            "deduplicationWindowTransitions": self.max_transitions,
            "droppedTransitionCount": self.dropped_transition_count,
            "recoveryInvocations": self.recovery_invocations,
            "recoveryInvocationCapacity": self.max_invocations,
            "retainedRecoveryInvocationCount": len(self.recovery_invocations),
            "observedRecoveryInvocationStartCount": self.recovery_invocation_count,
            "droppedRecoveryInvocationCount": self.dropped_recovery_invocation_count,
            "recoveryNodeClassifier": {
                "basis": "configured_exact_leaf_name_and_idle_departure",
                "nodeNames": sorted(self.recovery_node_names),
                "directTerminalNodeNames": sorted(self.direct_terminal_recovery_node_names),
                "directTerminalClassifierBasis": self.direct_terminal_classifier_basis,
                "directTerminalClassifierSha256": self.direct_terminal_classifier_sha256,
                "directTerminalInterpretation": (
                    "source_verified_leaf_completed_in_one_bt_tick; "
                    "not_proof_of_external_side_effect_or_physical_cause"
                ),
                "invocationStartPatterns": {
                    "allConfiguredLeaves": ["IDLE->RUNNING"],
                    "directTerminalLeaves": ["IDLE->SUCCESS"],
                },
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
                "restartWithoutResetRecoveryInvocationIds": restart_without_reset_ids,
                "messageLossDetectable": False,
                "exactRecoveryCountEligible": False,
                "limitations": reasons,
            },
        }
