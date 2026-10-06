"""Separate a planner's geometric rejection from unrelated navigation failures."""
import re


def planner_failure_evidence(controller_log, action_status, topology_result):
    """The grid is independent geometry evidence; log counts are observations only."""
    rejections = [line for line in controller_log.splitlines() if re.search(
        r"\[planner_server\].*plugin failed to plan.*Failed to create plan",
        line, re.IGNORECASE)]
    terminal_abort = bool(re.search(
        r"\[bt_navigator\].*Goal failed", controller_log, re.IGNORECASE))
    closed = (topology_result.get('expectedReachable') is False
              and topology_result.get('reachable') is False
              and topology_result.get('startBlocked') is False)
    checks = {
        'terminalActionAborted': action_status == 'aborted',
        'plannerRejectedGeometry': bool(rejections),
        'terminalNavigatorFailureLogged': terminal_abort,
        'authoritativeGeometryHasNoRoute': closed,
    }
    return {
        'checks': checks,
        'observedPlannerRejectionLines': len(rejections),
        'plannerRejectionExamples': rejections[:2] + rejections[-1:] if rejections else [],
        'topology': topology_result,
        'boundary': 'Log lines are observed rejection evidence, not exact planner invocation counts.',
    }
