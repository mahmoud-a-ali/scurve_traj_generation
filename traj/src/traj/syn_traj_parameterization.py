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


def calculate_jrk_for_time(Dp, J_sgn, Tph):
    #convet duration and jrk to 1d list
    jrk_dur_arr = np.array( ph_jrk_dur)
    J =  jrk_dur_arr[:,0].tolist()
    T =  jrk_dur_arr[:,1].tolist()
    
    print T, J
    #convert dur to  times
    T_ph = [ sum(T[0:i]) for i in range(0, len(T)+1) ]
       
    #calculate pos,vel,acc at each phase change (inflection points): 
    A=[0.0]
    V=[v_start]
    P=[p_start]
    for i in range (0, 10):
        A.append( J[i]*T[i]        +  A[i] )
        V.append( J[i]*T[i]**2/2.0 +  A[i]*T[i]          + V[i]  )
        P.append( J[i]*T[i]**3/6.0 +  A[i]*T[i]**2/2.0   + V[i]*T[i]  + P[i] )
        
    
    
    
    
    
def path_max_vel(path, v_start, v_end):
    # absolute values 
    abs_v_start = abs(v_start)
    abs_ve_end = abs(v_end)    
    # get po dir at each waypoint and indx for stop point where velocity changes its direction
    wpts_vel_dir, stp_idx= traj.set_stp_pts_to_zero(path)
    
    # convert path to positive path, so we can calculate velocity in one positive direction 
    mono_path = [ abs(pt) for pt in path]
    
    
    ### divide traj to small set of trajs 
    stp_idx = [0] + [id for id in stp_idx] + [ len(path)-1]
    path_set = [ mono_path[i:j+1] for i,j in zip(stp_idx[:-1], stp_idx[1:])]
    #print " len(path_set): {}".format(len(path_set))
    
    
    ## estimate velocity between each two stop points
    Estimated_vel = []
    Estimated_vel.append(abs_v_start)   
    for pth in range (0, len(path_set) ):
        waypts = path_set[pth]
        #print waypts
        n_segs = len(waypts) -1
        v_forward = 0.0
        v_backward= 0.0    
        
        if pth==0:  ##for the first potion of the traj, before the first stop point
            v_forward = abs_v_start
            v_backward = 0.0        
        if pth== len(path_set) -1 : ##for the last potion of the traj, after the last stop point
            v_forward = 0.0
            v_backward = abs_ve_end
            if len(path_set) -1 == 0 : ##if it does not have stop point 
                v_forward = abs_v_start
                v_backward = abs_ve_end
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
        #fig = plt.figure()    
        #plt.plot( forward_max_vel, 'b*')
        #plt.plot( forward_max_vel, 'bo', label='forward_vel')
        #plt.plot( backward_max_vel,  'r*')
        #plt.plot( backward_max_vel,  'ro', label='backwar_vel')
        #plt.xlabel("waypoints")
        #plt.ylabel("velocity")
        #plt.legend()
        #plt.show()
        
    
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
    return Estimated_vel








###################### start node, and set log_level to degub ######################
#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()
##### check 
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0



#trajectory waypoints, different traj 

path =[ [ -0.1748052395191716,-0.34961047903834325, -0.5244157185575147,-0.6992209580766863,-0.8740261975958581, -1.0488314371150294,-1.223636676634201,  -1.3984419161533728,  -1.5732471556725443], 
        [  0.1748052395191716,0.34961047903834325, 0.5244157185575147,0.6992209580766863,0.8740261975958581, 1.0488314371150294,1.223636676634201,  1.3984419161533728,  1.5732471556725443] ]   
path = [ [0.0, 0.2, 0.8, 2.0, 2.8, 1.3, 2.2, 3.50, 2.8, 3.5], 
         [0.5, 0.6, 3.2, 1.5, 0.9, 0.3, 0.8, 1.00, 2.0, 0.5] ]


#path = [ [0.5, 1.0, 2.5, 2.0,  2.5, 3.0, 4.5, 5.0, 7.5,  8.5], 
#         [0.2, 0.8, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4,  5.0]  ]

       
###### sub-sample/ dense path case
dense_path = []
for pth in path:
    dense_pth = []
    incr = np.linalg.norm( pth, ord=1)
    for wpt in range (0, len(pth)-1):
        sgn = math.copysign(1, (pth[wpt+1]-pth[wpt])  )
        new_segs= [ p for p in np.arange(pth[wpt], pth[wpt+1], sgn*5.0/incr) ]
        dense_pth = dense_pth + new_segs
    dense_path.append(dense_pth)
#path = dense_path
         
         
         

#start/end velocity of the trajectory
v_start = [ -0.5, 0.2]
v_end   = [ -0.2, 0.3]

### fins max vel per each waypoint
vel1 = path_max_vel(path[0], v_start[0], v_end[0] )
vel2 = path_max_vel(path[1], v_start[1], v_end[1] )

Estimated_vel = [ vel1, vel2 ]
for i in range(0, len(path)):
    print "len.Estimated_vel[{}]: {}".format(i, len(Estimated_vel[i]) ) 
### time to find final max velocity
#t_to_find_max_vel = time.time() - t_init
#print  t_to_find_max_vel
#print " len(path): {}".format(len(path))
#

#### plot estimated max vel for the all traj 
#fig = plt.figure()
#for i in range(0, len(path)):
#    plt.plot( Estimated_vel[i], '-', label='max_reachable_vel')
#    plt.plot( Estimated_vel[i], '*')
#    plt.legend(" estimated_max_vel ")
#    plt.xlabel("waypts")
#    plt.ylabel("velocity")
#    plt.legend()
#plt.show()
    
    
#exit()



##### fit segment according to path + estimated_vel
###use the previous non_zero segment planning to fit each segment of the trajetory with corresponding velocity
POS = []
VEL = []
ACC = []
JRK = [] 
TIM = []
T_SEG= []

dof= len(path)
#dof = 1
for i in range(0, dof): 
    Pos = []
    Vel = []
    Acc = []
    Jrk = [] 
    Tim = [] 
    T_seg= []
    # time and sampling 
    t_start = 0.0
    frq = 125.0
    T_seg=[ t_start]
    n_segs = len(path[i]) -1
    
    for seg in range(0, n_segs):
        #print "\n>> seg {}: path[seg]={}, path[seg+1]={}, Estimated_vel[seg]={}, Estimated_vel[seg+1]={}".format(seg, path[i][seg], path[i][seg+1], Estimated_vel[i][seg], Estimated_vel[i][seg+1])
        jrk_dur = traj.fit_traj_segment(path[i][seg], path[i][seg+1], Estimated_vel[i][seg], Estimated_vel[i][seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        
        #convert duration and jrk to 1d list
        jrk_dur_arr = np.array( jrk_dur)
        T =  jrk_dur_arr[:,1].tolist()
        
        #T_seg: contains the start/end time of each segment
        T_seg.append(  sum(T) + T_seg[seg] )
        t_start= T_seg[seg]
        t= t_start
#    ###### just to check time 
#    t = time.time() - t_init
#    print t 
#    print len(path)
#    exit()
#    for i in path:
        while t < T_seg[seg+1] :
            pos, vel, acc, jrk = traj.sample_segment(t, t_start, path[i][seg], Estimated_vel[i][seg], jrk_dur )    
    
            Pos.append(pos)
            Vel.append(vel* math.copysign(1, vel))
            Acc.append(acc)
            Jrk.append(jrk)
            Tim.append(t )
            t = t + (1.0/frq)
    POS.append(Pos)
    VEL.append(Vel)
    ACC.append(Acc)
    JRK.append(Jrk)
    TIM.append(Tim)
    T_SEG.append(T_seg)

#print "T_seg: {}".format(T_seg)
#print "path: {}".format(path)
#print "Estimated_vel: {}".format(Estimated_vel)
dof = len(path)
#dof = 1
for i in range(0, dof):
    Estimated_vel[i] = [ v*math.copysign(1, v) for v in Estimated_vel[i]]

T_arr = np.array(T_SEG  )
D_arr = np.diff(T_arr)
V_arr = np.array(Estimated_vel  )

print "\n>>> T: \n{}".format( np.rot90(T_arr,3) )
print "\n>>> D: \n{}".format( np.rot90(D_arr,3) )
print "\n>>> V: \n{}".format( np.rot90(V_arr,3) )
##################################### how to syn all the dof using Times not velocity 
### need to change to from fit vel to fit time !!!!!
### should have func to calculate jerk for certain time, or vel 
### no rule to synchronize, all of them should be projected in one dimension variable  

exit()
# convert path to positive path, so we can calculate velocity in one positive direction 


### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4, sharex=True)
traj_style = ['r-', 'b-', 'g-']
waypt_style = ['ro', 'bo', 'go']


for i in range(0, dof): 
    axes[0].plot( TIM[i], POS[i], traj_style[i])
    axes[0].plot( T_SEG[i], path[i], waypt_style[i])
axes[0].set_ylabel('position')
axes[0].grid()

for i in range(0,dof ):
    axes[1].plot( TIM[i], VEL[i], traj_style[i])
    axes[1].plot( T_SEG[i], Estimated_vel[i],  waypt_style[i])
axes[1].set_ylabel('velocity')
axes[1].grid()

for i in range(0, dof):
    axes[2].plot( TIM[i], ACC[i], traj_style[i])
axes[2].set_ylabel('acceleration')
axes[2].grid()

for i in range(0, dof):
    axes[3].plot( TIM[i], JRK[i], traj_style[i])
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')
axes[3].grid()
plt.legend()
plt.show()

#t_to_estimate_vel_and_fit_n_segs = time.time()    
#print ">> time to fit {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_fit_init)*1000.0)
#print ">> time to parameterize {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_init)*1000.0)
#exit()  

