from surface_evidence import transition_evidence


def rows(sequence):
    return [dict(leftSurface=s,rightSurface=s) for s in sequence for _ in range(3)]


def test_both_wheels_measured_on_all_transition_surfaces():
    d=transition_evidence(rows(['compacted-dirt','metal-plate','compacted-dirt','rubber-mat','painted-epoxy','normal-concrete']))
    assert all(d['checks'].values())


def test_goal_success_alone_cannot_establish_surface_traversal():
    assert not all(transition_evidence(rows(['compacted-dirt']))['checks'].values())


def test_wrong_order_or_one_wheel_missing_a_patch_fails():
    r=rows(['compacted-dirt','metal-plate','painted-epoxy','rubber-mat','normal-concrete'])
    assert not transition_evidence(r)['checks']['leftSurfaceObservedOrderedTransition']
    r=rows(['compacted-dirt','metal-plate','rubber-mat','painted-epoxy','normal-concrete'])
    for sample in r:sample['rightSurface']='compacted-dirt'
    assert not transition_evidence(r)['checks']['rightSurfaceObservedAllRequiredSurfaces']
