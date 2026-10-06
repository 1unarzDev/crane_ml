import math
from check_topology import connectivity
from generate_annex_trial import resolve,TASKS


def scenario(manifest,identifier='warehouse_compact_switchbacks'):
    return next(s for s in manifest['scenarios'] if s['id']==identifier)


def test_all_task_directions_are_reproducible_and_nontrivial():
    for task in TASKS:
        for seed in range(1,31):
            a=resolve(seed,'warehouse_compact_switchbacks',task,True)
            assert a==resolve(seed,'warehouse_compact_switchbacks',task,True)
            s=scenario(a)
            assert math.dist(s['start'],s['goal'])>=8
            assert abs(s['startYaw']-s['goalYaw'])<=4
            if task in ['return_to_receiving','maintenance_return','inspection_return']:
                assert s['start'][1]>s['goal'][1]


def test_seeded_task_selection_covers_multiple_actual_tasks():
    names={scenario(resolve(seed,'warehouse_compact_switchbacks','auto'))['routeName'] for seed in range(1,31)}
    assert len(names)==len(TASKS)


def test_all_task_pairs_have_clear_endpoints_and_feasible_paths():
    for task in TASKS:
        m=resolve(741,'warehouse_compact_switchbacks',task,True)
        result=connectivity(m,scenario(m))
        assert result['reachable'] and not result['startBlocked'] and not result['goalBlocked']
        assert result['gridPathLengthMeters']>=8


def test_reverse_no_path_retains_actual_disconnection():
    m=resolve(742,'warehouse_compact_no_path','return_to_receiving',True)
    result=connectivity(m,scenario(m,'warehouse_compact_no_path'))
    assert result['passCheck'] and not result['reachable']
    assert not result['startBlocked'] and not result['goalBlocked']


def test_parking_variation_keeps_narrow_passages_physically_open():
    for seed in range(1,31):
        s=scenario(resolve(seed,'warehouse_compact_switchbacks','auto',True));b={x['id']:x for x in s['obstacles']}
        bench=b['annex-packing-bench'];tote=b['annex-tote-cart'];tool=b['annex-tool-cart'];machine=b['annex-maintenance-machine']
        packing_gap=tote['center'][0]-tote['size'][0]/2-(bench['center'][0]+bench['size'][0]/2)
        machine_gap=machine['center'][0]-machine['size'][0]/2-(tool['center'][0]+tool['size'][0]/2)
        assert .81-1e-8<=packing_gap<=.89+1e-8
        assert .725-1e-8<=machine_gap<=.775+1e-8
