from no_path_evidence import planner_failure_evidence


GEOMETRY = dict(expectedReachable=False, reachable=False, startBlocked=False)
REJECTION = ('[WARN] [12] [planner_server]: GridBased plugin failed to plan from '
             '(16,0) to (24,0): "Failed to create plan with tolerance of: 0.200000"')
ABORT = '[ERROR] [15] [bt_navigator]: Goal failed'


def test_real_planner_rejection_and_closed_geometry_establish_reason():
    result = planner_failure_evidence(REJECTION + '\n' + ABORT, 'aborted', GEOMETRY)
    assert all(result['checks'].values())


def test_transport_failure_cannot_stand_in_for_planner_rejection():
    log = '[ERROR] [local_costmap]: Failed to activate because odom unavailable\n' + ABORT
    assert not planner_failure_evidence(log, 'aborted', GEOMETRY)['checks']['plannerRejectedGeometry']


def test_controller_failure_with_feasible_geometry_is_not_no_path():
    result = planner_failure_evidence(REJECTION + '\n' + ABORT, 'aborted',
                                     dict(GEOMETRY, reachable=True))
    assert not result['checks']['authoritativeGeometryHasNoRoute']


def test_timeout_and_invalid_spawn_do_not_pass():
    assert not planner_failure_evidence(REJECTION + '\n' + ABORT, 'timeout',
                                       GEOMETRY)['checks']['terminalActionAborted']
    assert not planner_failure_evidence(REJECTION + '\n' + ABORT, 'aborted',
                                       dict(GEOMETRY, startBlocked=True))['checks']['authoritativeGeometryHasNoRoute']
