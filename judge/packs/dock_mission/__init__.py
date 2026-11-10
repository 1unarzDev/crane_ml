"""The gate -> U-turn -> dock mission (DecisionCal): outcome metrics from ``truth.*`` and events only,
the same on the fast, integrated and physical layers.

  PACK, TASK          the judge pack and task spec
  facts(rec)          one pass of mission.run's rules over a record's truth
  outcome(rec)        docked / timeout_no_approach / timeout_in_slip / contact_fail / gate_miss / unscorable
  outcome_time(rec)   (category, t decided), for judge.insight.layers
  recompute_events    dock events from truth; event_check / event_insight compare them with the stored ones
"""

from judge.packs.dock_mission.pack import (DOCK_KINDS, METRICS, OUTCOMES, PACK, TASK, event_check, event_insight,
                                           facts, outcome, outcome_time, recompute_events)

__all__ = ['DOCK_KINDS', 'METRICS', 'OUTCOMES', 'PACK', 'TASK', 'event_check', 'event_insight', 'facts', 'outcome',
           'outcome_time', 'recompute_events']
