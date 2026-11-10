import sys
sys.path.insert(0,'/home/lunarz/worktrees/decisioncal')
import numpy as np
from plantcard.marine import mcap_lite as M
p='/home/lunarz/astro_dock/bags/field_trials_20261018/agent_battery_03_01_yaw_pos.files/bag/bag_0.mcap'
d=M.read(p,['/gps/fix','/imu/raw','/odom/local','/odom/mavros','/thruster_pwm_node/test_pulses'])
for k,v in d.items(): print(k,len(v), v[0][0] if v else None)
for k,dec in (('/gps/fix',M.navsatfix),('/imu/raw',M.imu),('/odom/local',M.odometry),('/odom/mavros',M.odometry)):
    lt=np.array([t for t,_ in d[k]]); st=np.array([dec(x)['stamp'] for _,x in d[k]])
    print(k,'stamp0',st[0],'log0',lt[0],'log-stamp mean %.4f sd %.4f min %.4f max %.4f'%((lt-st).mean(),(lt-st).std(),(lt-st).min(),(lt-st).max()),
      'dt_log med %.4f sd %.4f; dt_stamp med %.4f sd %.4f'%(np.median(np.diff(lt)),np.diff(lt).std(),np.median(np.diff(st)),np.diff(st).std()))
pw=[(t,M.uint16_array(x)) for t,x in d['/thruster_pwm_node/test_pulses']]
print(len(pw)); 
prev=None
for t,v in pw:
    if v!=prev: print(round(t-pw[0][0],4), v); prev=v
print([ (round(t-pw[0][0],3)) for t,v in pw[:12]])
