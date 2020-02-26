#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time
from rospkg import RosPack
import os


'''
example to parameterize trajectory by calculating the max_reaxchable_vel at the end of each segment starting with v_start to achieve pos_diff= p_end - p_start
currently wotking on +ve velocity only

'''

#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)

##### check 
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0


#start/end velocity of the trajectory
v_start = 0.8
v_end   = 0.5


#trajectory waypoints, different traj 
waypts = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.5, 5.0, 7.5,  8.5]
waypts = [0.2, .8, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4, 5.0]
waypts = [0.5, .8, 1.0, 1.2, 1.6, 2.0, 2.3, 2.5, 2.8, 3.0, 3.1, 3.4, 3.7, 4.2, 4.4, 5.0]

####consider sub-sampling same traj in such a way #waypoints =100, n_segs = 99 
sub_sampled_traj  = np.linspace(waypts[0], waypts[-1], 100)
waypts = sub_sampled_traj.tolist()

###consider trajectory with n equal-distance waypoints  
#sub_sampled_traj  = np.linspace( 2.0, 25.0, 1000)
#waypts = sub_sampled_traj.tolist()


##variables 
n_segs = len(waypts) -1
p_start = waypts[0]
v_forward = v_start
v_backward= v_end


#two vectors for forward/backward max_reachable_vel
forward_max_vel = [ v_forward ]
backward_max_vel = [ v_backward ]
vel_dir_vec = [ math.copysign( 1, (waypts[1] - waypts[0])  ) ]


##calculate execution time per function
t_calculate_seg_max_vel = []
t_to_fit_seg = []
t_to_sample = []


t_init = time.time()
### estimate max reachable velocity
for wpt in range(0, n_segs ) :
    t_max_vel_1 = time.time()
    tj, ta, tv, v_frwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[wpt], waypts[wpt+1], v_forward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    tj, ta, tv, v_bkwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[n_segs-wpt-1], waypts[n_segs-wpt], v_backward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    t_calculate_seg_max_vel.append( (time.time() - t_max_vel_1)/2.0 )
        
    forward_max_vel.append(  v_frwrd )
    backward_max_vel.append( v_bkwrd )
    v_forward = v_frwrd # to be used as v_start of the next segment
    v_backward = v_bkwrd

t_to_calculate_max_vel_n_segs= time.time() - t_init
backward_max_vel.reverse()
##check condition when v_start is not feasible: v_start > max_v_start calculated using the backward loop 
if v_start > backward_max_vel[0]:
    print "v_start should be <= {}".format( backward_max_vel[0] )
    raise ValueError("v_start is not feasible" )     

#fig = plt.figure()    
#plt.plot(waypts, forward_max_vel, 'b*')
#plt.plot(waypts, forward_max_vel, 'b', label='forward_vel')
#plt.plot(waypts, backward_max_vel,  'r*')
#plt.plot(waypts, backward_max_vel,  'r', label='backwar_vel')
#plt.xlabel("waypoints")
#plt.ylabel("velocity")
#plt.legend()


# calcuate max_rechable_vel that grantee grantee v_end at the end of the trajectory
estimated_vel =  [ min(v) for v in zip( forward_max_vel, backward_max_vel)] 
t_to_calculate_final_estimated_vel= time.time() - t_init

#fig = plt.figure()
#plt.plot(waypts, estimated_vel, 'g', label='max_reachable_vel')
#plt.plot(waypts, estimated_vel, 'g*')
#plt.legend(" estimated_max_vel ")
#plt.xlabel("waypts")
#plt.ylabel("velocity")
#plt.legend()
#plt.show()


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

t_fit_init = time.time()
for seg in range(0, n_segs):
    t_fit_seg_1 =  time.time()
    jrk_dur = traj.fit_traj_segment(waypts[seg], waypts[seg+1], estimated_vel[seg], estimated_vel[seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
    t_to_fit_seg.append( time.time() - t_fit_seg_1 )
    
    #convert duration and jrk to 1d list
    jrk_dur_arr = np.array( jrk_dur)
    T =  jrk_dur_arr[:,1].tolist()
    
    #T_seg: contains the start/end time of each segment
    T_seg.append(  sum(T) + T_seg[seg] )
    t_start= T_seg[seg]
    t= t_start
    
    while t < T_seg[seg+1] :
        t_to_sample_1 = time.time()
        pos, vel, acc, jrk = traj.sample_segment(t, t_start, p_start, v_start, jrk_dur )    
        t_to_sample.append( time.time() - t_to_sample_1)
        
        POS.append(pos)
        VEL.append(vel)
        ACC.append(acc)
        JRK.append(jrk)
        Tim.append(t )
        t = t + (1.0/frq)
    ##calculate next p_start, v_start: could you sample or use waypt[seg+1] and estimated_vel[seg+1]
    p_start, v_start, acc, jrk = traj.sample_segment(T_seg[seg+1], t_start, p_start, v_start, jrk_dur )
    p_start=waypts[seg+1]
    v_start=estimated_vel[seg+1]

t_end = time.time()

# convert to millisec
t_calculate_seg_max_vel = [x*1000.0 for x in t_calculate_seg_max_vel ]
t_to_fit_seg = [x*1000.0 for x in t_to_fit_seg ]
t_to_sample = [x*1000.0 for x in t_to_sample ]



#calculate average 
avg_t_calculate_seg_max_vel = sum(t_calculate_seg_max_vel )/ float( len(t_calculate_seg_max_vel) )
avg_t_to_fit_seg = sum(t_to_fit_seg )/ float( len(t_to_fit_seg) )
avg_t_to_sample = sum(t_to_sample )/ float( len(t_to_sample) )

### print times/statistics to file
#rospack1 = RosPack()
#traj_pkg_path = rospack1.get_path('traj')
#new_dir = traj_pkg_path + "/Figs/timing_info_{}_waypts/".format(n_segs+1)
#if not os.path.exists(new_dir):
#    os.makedirs(new_dir)
#timing_info_file =  new_dir + "exec_time_{}_waypts.txt".format(n_segs+1) #bo_airport.xml"
#print timing_info_file
#with open(timing_info_file, 'w') as f:
#    print >> f, ">> n_segs= {} segment".format(n_segs)
#    print >> f, ">> avg_t_calculate_seg_max_vel= {} msec".format(avg_t_calculate_seg_max_vel)
#    print >> f, ">> avg_t_to_fit_seg= {} msec".format(avg_t_to_fit_seg)
#    print >> f, ">> avg_t_to_sample = {} msec \n".format( avg_t_to_sample)   
#    print >> f, ">> predictable time to estimate max_vel for {} waypts =  {} msec ".format( n_segs+1,  2*n_segs*avg_t_calculate_seg_max_vel)    
#    print >> f, ">> calculated  time to estimate max_vel for {} waypts =   {} msec \n".format( n_segs+1,  t_to_calculate_max_vel_n_segs*1000.0)
#    print >> f, ">> total time of parameterization (find_max_vel, and fit_non-zero segment), and sampling of {} waypoints= {} msec \n".format( n_segs+1,  (t_end-t_init)*1000.0  )


#fig = plt.figure()
#plt.plot(t_calculate_seg_max_vel, 'r.',  label='t_per_seg')
#plt.title('time to calculate max_vel per waypoint' )
#plt.ylabel('Time milli-sec')
#plt.xlabel('segment_number')
#plt.axhline(y= avg_t_calculate_seg_max_vel, color='g', linestyle='--', label='avg_time')
#plt.legend()
#
#fig = plt.figure()
#plt.plot(t_to_fit_seg, 'r.', label='t_per_seg')
#plt.title("time to fit seg" )
#plt.ylabel('Time milli-sec')
#plt.xlabel('segment_number')
#plt.axhline(y= avg_t_to_fit_seg, color='g', linestyle='--', label='avg_time')
#plt.legend()
#
#fig = plt.figure()
#plt.plot(t_to_sample, 'r.', label='t_per_sample')
#plt.title("time to sample a traj at instant: t" )
#plt.ylabel('Time milli-sec')
#plt.xlabel('sampling instant')
#plt.axhline(y= avg_t_to_sample, color='g', linestyle='--', label='avg_time')
#plt.legend()



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
#plt.show()

#t_to_estimate_vel_and_fit_n_segs = time.time()    
#print ">> time to fit {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_fit_init)*1000.0)
#print ">> time to parameterize {} non_zero segs = {} msec".format( n_segs, ( t_to_estimate_vel_and_fit_n_segs - t_init)*1000.0)
#exit()  

