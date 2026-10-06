"""Known-map ground rejection; raw scan acquisition remains authoritative.

This requires a surveyed, registered terrain prior. It uses measured scan ranges
and TF, never raycast normals, collider IDs or the robot's intended trajectory.
"""
import math


def rotate(point, quaternion):
    x,y,z=point;qx,qy,qz,qw=quaternion
    tx=2*(qy*z-qz*y);ty=2*(qz*x-qx*z);tz=2*(qx*y-qy*x)
    return (x+qw*tx+qy*tz-qz*ty,y+qw*ty+qz*tx-qx*tz,z+qw*tz+qx*ty-qy*tx)


def terrain_return(point, surfaces, tolerance=.008):
    """Reject upward floor faces only; vertical platform sides remain obstacles."""
    # ROS odom FLU -> Unity map axes.
    wx,wy,wz=-point[1],point[2],point[0]
    for box in surfaces:
        if box['kind'] not in ('floor','ramp'):continue
        dx=wx-box['center'][0];dy=wy-box['center'][1];dz=wz-box['center'][2]
        yaw=math.radians(box.get('yaw',0));pitch=math.radians(box.get('pitch',0))
        # Inverse of Unity rotation Y(yaw) X(pitch).
        lx=math.cos(yaw)*dx-math.sin(yaw)*dz
        yz=math.sin(yaw)*dx+math.cos(yaw)*dz
        ly=math.cos(pitch)*dy+math.sin(pitch)*yz
        lz=-math.sin(pitch)*dy+math.cos(pitch)*yz
        # Retain boundary returns: an upper slab side can be within the height
        # tolerance of its top face. A conservative 2 mm interior margin avoids
        # labelling those vertical faces as traversable ground.
        edge_margin=min(.002,tolerance)
        if abs(lx)<box['size'][0]/2-edge_margin and abs(lz)<box['size'][2]/2-edge_margin and abs(ly-box['size'][1]/2)<=tolerance:
            return True
    return False


def filter_ranges(ranges, angle_min, angle_increment, translation, quaternion, surfaces):
    filtered=[];removed=0
    for index,distance in enumerate(ranges):
        if not math.isfinite(distance):filtered.append(distance);continue
        angle=angle_min+index*angle_increment
        point=rotate((distance*math.cos(angle),distance*math.sin(angle),0),quaternion)
        point=tuple(v+t for v,t in zip(point,translation))
        ground=terrain_return(point,surfaces)
        filtered.append(float('inf') if ground else distance);removed+=int(ground)
    return filtered,removed
