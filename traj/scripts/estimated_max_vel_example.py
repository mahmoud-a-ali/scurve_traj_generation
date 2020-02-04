#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np

'''
example to parameterize trajectory by calculating the max_reaxchable_vel at the end of each segment starting with v_start to achieve pos_diff= p_end - p_start
'''

#set log_level to debug
rospy.init_node('traj_segment_node', log_level=rospy.DEBUG)

##### check 
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0


p_start = 0.0
p_end   = 1.0
v_start = 1.5
v_end   = 0.0


#case of waypoint 
waypts = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.5, 5.0, 7.5,  8.5]
waypts = [0.5, .8, 1.0, 1.2, 1.6, 2.0, 2.3, 2.5, 2.8, 3.0, 3.1, 3.4, 3.7, 4.2, 4.4, 5.0]
waypts = [0.2, .8, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4, 5.0]


n_segs = len(waypts) -1
v_forward = v_start
v_backward= v_end

#two vectors for forward/backward max_reachable_vel
forward_max_vel = [ v_forward ]
backward_max_vel = [ v_backward ]
vel_dir_vec = [ math.copysign( 1, (waypts[1] - waypts[0])  ) ]

### estimate max reachable velocity
for wpt in range(0, n_segs ) :
    tj, ta, tv, v_frwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[wpt], waypts[wpt+1], v_forward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    tj, ta, tv, v_bkwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[n_segs-wpt-1], waypts[n_segs-wpt], v_backward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)

    forward_max_vel.append(  v_frwrd )
    backward_max_vel.append( v_bkwrd )
    v_forward = v_frwrd # to be used as v_start of the next segment
    v_backward = v_bkwrd

backward_max_vel.reverse()
plt.plot(waypts, forward_max_vel, 'b*')
plt.plot(waypts, forward_max_vel, 'b', label='forward_vel')
plt.plot(waypts, backward_max_vel,  'r*')
plt.plot(waypts, backward_max_vel,  'r', label='backwar_vel')
plt.xlabel("waypoints")
plt.ylabel("velocity")
plt.legend()
plt.show()


# calcuate max_rechable_vel that grantee grantee v_end at the end of the trajectory
estimated_vel =  [ min(v) for v in zip( forward_max_vel, backward_max_vel)] 
print "estimated_max_vel= {}".format(estimated_vel)
plt.plot(waypts, estimated_vel, 'g', label='max_reachable_vel')
plt.plot(waypts, estimated_vel, 'g*')
plt.legend(" estimated_max_vel ")
plt.xlabel("waypts")
plt.ylabel("velocity")
plt.legend()
plt.show()



###use the previous non_zero segment planning to fit each segment of the trajetory with corresponding velocity
POS = []
VEL = []
ACC = []
JRK = [] 

# time and sampling 
Tim =[]
t_start = 0.0
frq = 125.0
T_seg=[ t_start]

for seg in range(0, n_segs):
    jrk_dur = traj.fit_traj_segment(waypts[seg], waypts[seg+1], estimated_vel[seg], estimated_vel[seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    print jrk_dur
    #convet duration and jrk to 1d list
    jrk_dur_arr = np.array( jrk_dur)
    T =  jrk_dur_arr[:,1].tolist()
    
    #T_seg: contains the start/end time of each segment
    T_seg.append(  sum(T) + T_seg[seg] )
    t_start= T_seg[seg]
    t= t_start

    while t < T_seg[seg+1] :
        pos, vel, acc, jrk = traj.sample_segment(t, t_start, p_start, v_start, jrk_dur )   
        POS.append(pos)
        VEL.append(vel)
        ACC.append(acc)
        JRK.append(jrk)
        Tim.append(t )
        t = t + (1.0/frq)
    p_start = pos
    v_start = vel

fig, axes = plt.subplots(4, sharex=True)
axes[0].plot( Tim, POS)
axes[0].set_ylabel('position')
axes[1].plot( Tim, VEL)
axes[1].set_ylabel('velocity')
axes[2].plot( Tim, ACC)
axes[2].set_ylabel('acceleration')
axes[3].plot( Tim, JRK)
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')

plt.legend()
plt.show()



