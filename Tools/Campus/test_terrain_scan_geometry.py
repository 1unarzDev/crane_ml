import math
from terrain_scan_geometry import filter_ranges,terrain_return


def box(kind='floor',center=(0,-.1,0),size=(10,.2,10),pitch=0):
    return dict(kind=kind,center=center,size=size,pitch=pitch,yaw=0)


def test_floor_face_rejected_but_low_obstacle_and_platform_side_retained():
    floor=box();slab=box(center=(0,.175,2),size=(3,.35,3))
    assert terrain_return((0,0,0),[floor])
    assert not terrain_return((0,0,.012),[floor])
    assert not terrain_return((2,0,.18),[slab])
    assert terrain_return((2,0,.35),[slab])


def test_sloped_face_matches_registered_map():
    ramp=box('ramp',(9,.074692,35),(3,.2,4),-5)
    assert terrain_return((35,-9,.074692+math.cos(math.radians(5))*.1),[ramp])
    assert not terrain_return((35,-9,.25),[ramp])


def test_level_scan_is_preserved_and_nan_inf_are_preserved():
    values,removed=filter_ranges([1,float('inf'),float('nan')],0,1,(0,0,.2),(0,0,0,1),[box()])
    assert removed==0 and values[0]==1 and math.isinf(values[1]) and math.isnan(values[2])


def test_measured_pitch_and_translation_project_ground_return():
    angle=math.pi/4
    values,removed=filter_ranges([math.sqrt(2)],0,1,(0,0,1),(0,math.sin(angle/2),0,math.cos(angle/2)),[box()])
    assert removed==1 and math.isinf(values[0])
