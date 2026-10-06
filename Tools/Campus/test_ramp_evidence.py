from ramp_evidence import ramp_traversal


def trial(height=.36,x=9):
    manifest=dict(boxes=[dict(id='ground',kind='floor',center=[9,-.1,35],size=[20,.2,20]),
                         dict(id='platform',kind='floor',center=[9,.175,38.5],size=[3,.35,3])])
    scenario=dict(start=[9,31],goal=[9,39])
    rows=[dict(position=dict(x=9,y=.01,z=31))]+[dict(position=dict(x=x,y=height,z=39)) for _ in range(3)]
    return manifest,scenario,rows


def test_flat_route_cannot_satisfy_ramp_gate():
    d=ramp_traversal(*trial(.01))
    assert not d['checks']['measuredRaisedPlatformTraversal']


def test_supported_raised_goal_samples_pass():
    assert all(ramp_traversal(*trial())['checks'].values())


def test_height_without_platform_footprint_cannot_pass():
    assert not ramp_traversal(*trial(x=7))['checks']['robotFinishedOnRaisedGoalSurface']
