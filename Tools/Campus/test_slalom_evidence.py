from slalom_evidence import slalom_evidence


def trial(xs=(3.45,-3.45,3.45)):
    def box(name,x,z,sx,sz):return dict(id=name,center=[x,.45,z],size=[sx,.9,sz],yaw=0)
    # 8 m inside width; 1.1 m alternating clear gaps.
    obstacles=[box('prove-course-left',-4.125,4,.25,10),box('prove-course-right',4.125,4,.25,10)]
    banks=[box('slalom-bank-0',-.55,2,6.9,.5),box('slalom-bank-1',.55,4,6.9,.5),box('slalom-bank-2',-.55,6,6.9,.5)]
    rows=[]
    for index,x in enumerate(xs):
        z=2+index*2
        rows.extend([dict(position=dict(x=x,z=z-.01),simulationTime=index*2),dict(position=dict(x=x,z=z+.01),simulationTime=index*2+.1)])
    return dict(boxes=banks),dict(obstacles=obstacles),rows


def test_actual_alternating_crossings_pass():
    assert all(slalom_evidence(*trial())['checks'].values())


def test_straight_route_beside_bank_does_not_prove_slalom():
    result=slalom_evidence(*trial((3.45,3.45,3.45)))
    assert not result['checks']['gate1CrossedInsideFootprintClearance']


def test_missing_forward_crossing_cannot_be_inferred_from_goal():
    manifest,scenario,rows=trial();rows=rows[:4]
    assert not slalom_evidence(manifest,scenario,rows)['checks']['gate2MeasuredForwardCrossing']


def test_sparse_samples_do_not_prove_physical_gap_traversal():
    manifest,scenario,rows=trial();rows[1]['simulationTime']=1
    assert not slalom_evidence(manifest,scenario,rows)['checks']['gate0AcquisitionGapBounded']


def test_wide_opening_or_disconnected_bank_is_rejected():
    manifest,scenario,rows=trial();manifest['boxes'][0]['size'][0]=5
    assert not slalom_evidence(manifest,scenario,rows)['checks']['gate0IsTightPhysicalGap']
