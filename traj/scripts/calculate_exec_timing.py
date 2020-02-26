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
v_end   = 0.0

#trajectory waypoints, different traj 
waypts = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.5, 5.0, 7.5,  8.5]
waypts = [0.2, .8, 1.0, 1.2,  2.5, 2.8, 3.0, 4.2, 4.4, 5.0]
waypts = [0.5, .8, 1.0, 1.2, 1.6, 2.0, 2.3, 2.5, 2.8, 3.0, 3.1, 3.4, 3.7, 4.2, 4.4, 5.0]

####consider sub-sampling same traj in such a way #waypoints =100, n_segs = 99 
sub_sampled_traj  = np.linspace(waypts[0], waypts[-1], 100)
waypts = sub_sampled_traj.tolist()

###consider trajectory with n equal-distance waypoints  
sub_sampled_traj  = np.linspace( 2.0, 25.0, 1000)
waypts = sub_sampled_traj.tolist()

##variables 
n_segs = len(waypts) -1
p_start = waypts[0]
v_forward = v_start
v_backward= v_end



### vector to calculate average timing
Avg_seg_max_vel_time =[]
Avg_fit_seg_time =[]
T_to_max_vel_n_seg =[] 
T_to_fit_n_segs=[]
T_to_parameterize_n_segs=[]

n_exec = 1000
for i in range(0, n_exec):
        
    #two vectors for forward/backward max_reachable_vel
    forward_max_vel = [ v_forward ]
    backward_max_vel = [ v_backward ]
    vel_dir_vec = [ math.copysign( 1, (waypts[1] - waypts[0])  ) ]


    ##calculate execution time per time
    t_to_seg_max_vel = []
    t_to_fit_seg = []
    
    t_init = time.time()
    ### estimate max reachable velocity
    for wpt in range(0, n_segs ) :
        t_max_vel_1 = time.time()
        tj, ta, tv, v_frwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[wpt], waypts[wpt+1], v_forward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        tj, ta, tv, v_bkwrd, vel_dir = traj.max_reachable_vel_per_segment( waypts[n_segs-wpt-1], waypts[n_segs-wpt], v_backward, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        t_to_seg_max_vel.append( (time.time() - t_max_vel_1)/2.0 )
            
        forward_max_vel.append(  v_frwrd )
        backward_max_vel.append( v_bkwrd )
        v_forward = v_frwrd # to be used as v_start of the next segment
        v_backward = v_bkwrd
    
    backward_max_vel.reverse()
    ##check condition when v_start is not feasible: v_start > max_v_start calculated using the backward loop 
    if v_start > backward_max_vel[0]:
        print "v_start should be <= {}".format( backward_max_vel[0] )
        raise ValueError("v_start is not feasible" )     
    
    
    # calcuate max_rechable_vel that grantee grantee v_end at the end of the trajectory
    estimated_vel =  [ min(v) for v in zip( forward_max_vel, backward_max_vel)] 
    t_to_max_vel_n_seg = time.time() - t_init
    
  
    t_fit_init = time.time()
    for seg in range(0, n_segs):
        t_fit_seg_1 =  time.time()
        jrk_dur = traj.fit_traj_segment(waypts[seg], waypts[seg+1], estimated_vel[seg], estimated_vel[seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        t_to_fit_seg.append( time.time() - t_fit_seg_1 )
        
    t_to_fit_n_segs = time.time() - t_fit_init      
    t_to_parameterize_n_segs  = time.time() - t_init
    
    
    # convert to millisec
    t_to_seg_max_vel = [x*1000.0 for x in t_to_seg_max_vel ]
    t_to_fit_seg = [x*1000.0 for x in t_to_fit_seg ]
    
    
    
    #calculate average 
    avg_t_to_seg_max_vel = sum(t_to_seg_max_vel )/ float( len(t_to_seg_max_vel) )
    avg_t_to_fit_seg = sum(t_to_fit_seg )/ float( len(t_to_fit_seg) )

    Avg_seg_max_vel_time.append( avg_t_to_seg_max_vel )
    Avg_fit_seg_time.append( avg_t_to_fit_seg )
 
    
    T_to_max_vel_n_seg.append( t_to_max_vel_n_seg*1000.0 )
    T_to_fit_n_segs.append(  t_to_fit_n_segs*1000.0 )
    T_to_parameterize_n_segs.append(  t_to_parameterize_n_segs*1000.0 )



### calculate average over n times 
avg_t_to_seg_max_vel = sum(Avg_seg_max_vel_time )/ float( len(Avg_seg_max_vel_time) )
avg_t_to_fit_seg = sum(Avg_fit_seg_time )/ float( len(Avg_fit_seg_time) )

avg_t_to_max_vel_n_seg = sum(T_to_max_vel_n_seg )/ float( len(T_to_max_vel_n_seg) )
avg_t_to_fit_n_segs = sum(T_to_fit_n_segs )/ float( len(T_to_fit_n_segs) )
avg_t_to_parameterize_n_segs = sum(T_to_parameterize_n_segs )/ float( len(T_to_parameterize_n_segs) )



### print times/statistics to file
rospack1 = RosPack()
traj_pkg_path = rospack1.get_path('traj')
new_dir = traj_pkg_path + "/scripts/".format(n_segs+1)
if not os.path.exists(new_dir):
    os.makedirs(new_dir)
timing_info_file =  new_dir + "statistics_{}_exec_time_of_{}_waypts.txt".format(n_exec, n_segs+1) #bo_airport.xml"
print timing_info_file
with open(timing_info_file, 'w') as f:
    print >> f, ">> n_segs= {} segment, n_exec ={} \n".format(n_segs, n_exec)
    
    print >> f, "####### average per segment: "
    print >> f, ">> average time to calculate max_vel per segment =  {} msec".format(avg_t_to_seg_max_vel)
    print >> f, ">> average time to fit non-zero segment          =  {} msec \n".format(avg_t_to_fit_seg)

    print >>f, "####### average per {} segment: ".format(n_segs)
    
    print >> f, ">> average     time to calculate max_vel per {} seg  =  {} msec ".format( n_segs+1,  avg_t_to_max_vel_n_seg) 
    print >> f, ">> predictable time to calculate max_vel for {} segs =  {} msec \n".format( n_segs+1,  2*n_segs*avg_t_to_seg_max_vel)    

    print >> f, ">> average     time to fit {} non-zero segment       =  {} msec ".format( n_segs+1, avg_t_to_fit_n_segs)     
    print >> f, ">> predictable time to fit {} non-zero segment       =  {} msec \n".format( n_segs+1,  n_segs*avg_t_to_fit_seg)    

    print >> f, ">> avgerage    time to parameterize (find max_vel and time) for {} segs =  {} msec ".format( n_segs+1, avg_t_to_parameterize_n_segs) 
    print >> f, ">> predicatble time to parameterize (find max_vel and time) for {} segs =  {} msec \n".format( n_segs+1, 2*n_segs*avg_t_to_seg_max_vel+n_segs*avg_t_to_fit_seg) 








