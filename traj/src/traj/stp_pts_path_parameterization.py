#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time

'''
example to parameterize trajectory by calculating the max_reaxchable_vel at the end of each segment starting with v_start to achieve pos_diff= p_end - p_start
currently wotking on +ve velocity only

'''

#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()
##### check 
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0


#start/end velocity of the trajectory
v_start = 0.2
v_end   = 0.3


#trajectory waypoints, different traj 
path = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.5, 5.0, 7.5,  8.5]
path = [0.2, .8, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4, 5.0]
path = [0.5, .8, 1.0, 1.2, 1.6, 2.0, 2.3, 2.5, 2.8, 3.0, 3.1, 3.4, 3.7, 4.2, 4.4, 5.0]
path= [0.0, 0.4, 1.2, 5.0, 0.9, 0.3, 0.8, 1.00, 2.0, 0.5, -5.5, -1.0, 0.0]

path =  [ 0.1748052395191716,0.34961047903834325, 0.5244157185575147,0.6992209580766863,0.8740261975958581, 1.0488314371150294,1.223636676634201,  1.3984419161533728,  1.5732471556725443] 

## sub-sample/ dense path case
dense_path = []
incr = np.linalg.norm( path, ord=1)
for wpt in range (0, len(path)-1):
    sgn = math.copysign(1, (path[wpt+1]-path[wpt])  )
    new_segs= [ p for p in np.arange(path[wpt], path[wpt+1], sgn*5.0/incr) ]
    dense_path = dense_path + new_segs
#path = dense_path

# get po dir at each waypoint and indx for stop point where velocity changes its direction
wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)

# convert path to positive path, so we can calculate velocity in one positive direction 
mono_path = [ abs(pt) for pt in path]


### divide traj to small set of trajs 
stp_idx = [0] + [id for id in stp_idx] + [ len(path)-1]
path_set = [ mono_path[i:j+1] for i,j in zip(stp_idx[:-1], stp_idx[1:])]
print " len(path): {}".format(len(path))


## estimate velocity between each two stop points
Estimated_vel = []
Estimated_vel.append(v_start)   
for pth in range (0, len(path_set) ):
    waypts = path_set[pth]
    #print waypts
    n_segs = len(waypts) -1
    v_forward = 0.0
    v_backward= 0.0    
    
    if pth==0:  ##for the first potion of the traj, before the first stop point
        v_forward = v_start
        v_backward = 0.0        
    if pth== len(path_set) -1 : ##for the last potion of the traj, after the last stop point
        v_forward = 0.0
        v_backward = v_end
    if len(path_set) -1 == 0 : ##if it does not have stop point 
        v_forward = v_start
        v_backward = v_end
    #two vectors for forward/backward max_reachable_vel
    forward_max_vel = [ v_forward ]
    backward_max_vel = [ v_backward ]
    
    ### estimate max reachable velocity
    for wpt in range(0, n_segs ) :
        tj, ta, tv, v_frwrd = traj.max_reachable_vel( abs(waypts[wpt] - waypts[wpt+1]), v_forward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(waypts[wpt] - waypts[wpt+1]), v_forward, v_frwrd)        
        tj, ta, tv, v_bkwrd = traj.max_reachable_vel( abs(waypts[n_segs-wpt]- waypts[n_segs-wpt-1]), v_backward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        #print "abs_pos_diff={}, v_forward={}, nxt_v={} ".format(abs(waypts[n_segs-wpt]- waypts[n_segs-wpt-1]), v_backward, v_bkwrd)        

        forward_max_vel.append(  v_frwrd )
        backward_max_vel.append( v_bkwrd )
        v_forward = v_frwrd # to be used as v_start of the next segment
        v_backward = v_bkwrd
    
    backward_max_vel.reverse()
    fig = plt.figure()    
    plt.plot( forward_max_vel, 'b*')
    plt.plot( forward_max_vel, 'bo', label='forward_vel')
    plt.plot( backward_max_vel,  'r*')
    plt.plot( backward_max_vel,  'ro', label='backwar_vel')
    plt.xlabel("waypoints")
    plt.ylabel("velocity")
    plt.legend()
    plt.show()
    

    ##check condition when v_start is not feasible: v_start > max_v_start calculated using the backward loop 
    if forward_max_vel[0] > backward_max_vel[0]:
        print "v_start should be <= {}".format( backward_max_vel[0] )
        raise ValueError("v_start is not feasible" )     
        
    # calcuate max_rechable_vels that grantee v_end at the end of the trajectory for this portion of traj
    estimated_vel =  [ min(v) for v in zip( forward_max_vel, backward_max_vel)] 
    #accumlate the estimated velocity per each portion till complete estimated vel for all portions/waypoints 
    Estimated_vel = Estimated_vel + [v for v in estimated_vel[1:] ]

## retrieve the direction at each way point 
Estimated_vel=[v*dir for v, dir in zip(Estimated_vel, wpts_vel_dir) ]
### time to find final max velocity
t_to_find_max_vel = time.time() - t_init
print  t_to_find_max_vel
print " len(path): {}".format(len(path))


#### plot estimated max vel for the all traj 
fig = plt.figure()
plt.plot( Estimated_vel, 'g', label='max_reachable_vel')
plt.plot( Estimated_vel, 'g*')
plt.legend(" estimated_max_vel ")
plt.xlabel("waypts")
plt.ylabel("velocity")
plt.legend()
plt.show()
    
    



##### fit segment according to path + estimated_vel
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
n_segs = len(path) -1

for seg in range(0, n_segs):
    #print "\n>> seg {}: path[seg]={}, path[seg+1]={}, Estimated_vel[seg]={}, Estimated_vel[seg+1]={}".format(seg, path[seg], path[seg+1], Estimated_vel[seg], Estimated_vel[seg+1])
    jrk_dur = traj.fit_traj_segment(path[seg], path[seg+1], Estimated_vel[seg], Estimated_vel[seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    
    #convert duration and jrk to 1d list
    jrk_dur_arr = np.array( jrk_dur)
    T =  jrk_dur_arr[:,1].tolist()
    
    #T_seg: contains the start/end time of each segment
    T_seg.append(  sum(T) + T_seg[seg] )
    t_start= T_seg[seg]
    t= t_start

###### just to check time 
#t = time.time() - t_init
#print t 
#print len(path)
#exit()
#def aha():    
    while t < T_seg[seg+1] :
        pos, vel, acc, jrk = traj.sample_segment(t, t_start, path[seg], Estimated_vel[seg], jrk_dur )    

        POS.append(pos)
        VEL.append(vel)
        ACC.append(acc)
        JRK.append(jrk)
        Tim.append(t )
        t = t + (1.0/frq)


#print "T_seg: {}".format(T_seg)
#print "path: {}".format(path)
#print "Estimated_vel: {}".format(Estimated_vel)


### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4, sharex=True)
axes[0].plot( Tim, POS)
axes[0].plot( T_seg, path, 'ro')
axes[0].set_ylabel('position')
axes[0].grid()
axes[1].plot( Tim, VEL)
axes[1].plot( T_seg, Estimated_vel, 'ro')
axes[1].set_ylabel('velocity')
axes[1].grid()
axes[2].plot( Tim, ACC)
axes[2].set_ylabel('acceleration')
axes[2].grid()
axes[3].plot( Tim, JRK)
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')
axes[3].grid()
plt.legend()
plt.show()

#t_to_estimate_vel_and_fit_n_segs = time.time()    
#print ">> time to fit {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_fit_init)*1000.0)
#print ">> time to parameterize {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_init)*1000.0)
#exit()  

