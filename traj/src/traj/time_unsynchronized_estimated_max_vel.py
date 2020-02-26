#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time

'''
example to calculate the max_reaxchable_vel per each dof starting with v_start to end with v_end
currently: ndof are not time-synchronized yet

'''


    
#set log_level to debug
rospy.init_node('traj_segment_node')#, log_level=rospy.DEBUG)
t_init = time.time()

##### max values
abs_max_pos = 30.0
abs_max_vel = 3.0
abs_max_acc = 4.0
abs_max_jrk = 10.0


##old path from yaml file, total time for that traj is 3.5 
positions= [] 
positions.append( [0.001012041720806261, -0.0041338839998457414, -0.3364954637375829, 3.1335235418029623, -1.9007831579863803, -0.0016187068912901527]  )
positions.append( [0.0009791899982615775, -0.004000142678524483, -0.336361429598184, 3.1335236977034997, -1.900782568953615, -0.0016515062354158705]    )
positions.append( [-0.010225007491410358, 0.041612813371950455, -0.2906486068948853, 3.133576868139441, -1.9005816772219828, -0.01283783987793361]      )
positions.append( [-0.02146205670362698, 0.08735951074374665, -0.24480175005218777, 3.1336301944759195, -1.9003801964575855, -0.02405697286457707]      )
positions.append( [-0.032699105915843595, 0.13310620811554283, -0.19895489320949022, 3.133683520812398, -1.900178715693188, -0.03527610585122053]       )
positions.append( [-0.04393615512806022, 0.17885290548733904, -0.15310803636679265, 3.1337368471488767, -1.8999772349287907, -0.04649523883786398]      )
positions.append(  [-0.05517320434027684, 0.22459960285913524, -0.10726117952409508, 3.1337901734853553, -1.8997757541643931, -0.05771437182450745]     )
positions.append(  [-0.06641025355249344, 0.27034630023093137, -0.06141432268139757, 3.133843499821834, -1.8995742733999958, -0.0689335048111509]       )
positions.append( [-0.07764730276471007, 0.31609299760272763, -0.015567465838699945, 3.1338968261583124, -1.8993727926355983, -0.08015263779779436]     )
positions.append(  [-0.08888435197692669, 0.3618396949745238, 0.03027939100399757, 3.133950152494791, -1.899171311871201, -0.09137177078443781]         )
positions.append( [-0.10008854946659862, 0.4074526510249987, 0.07599221370729628, 3.1340033229307322, -1.8989704201395687, -0.10255810442695557]        )
positions.append( [-0.10012140118914331, 0.40758639234632, 0.07612624784669514, 3.1340034788312696, -1.8989698311068035, -0.10259090377108128]          )
#positions.append( [0.0 for pt in range (0, 6) ]          )


# Jon's path
positions =[]
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [ 1.5, 0.7, 0.3, 0.0, 0.0, 0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )
positions.append( [-1.5, 0.7, 0.3, 0.0, 0.0,0.0] )
positions.append( [ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] )


path_np = np.array( positions )
path = np.rot90( path_np ).tolist()
path.reverse()
 

#path =[ [0.5, 1.0, 1.5, 3.0, 2.2, 1.7, 0.5, 1.8, 3.1,  4.5], [0.2, .6, 1.0, 1.5,  1.1, 1.6, 2.0, 1.8, 1.6, 1.4]  ]
path = [[0.0, 1.0, -1.0, -1.7, 2.7], [0.0, 1.0, 1.5, 1.7, 2.7]]

n_jts  = len(path)
n_wpts = len(path[0])
n_segs = n_wpts - 1
print n_jts, n_wpts, n_segs  

#start/end velocity of the trajectory
v_start = [0.0 for pt in range (0, n_jts) ]
v_end   = [0.0 for pt in range (0, n_jts) ]

n_jts  = len(path)
n_wpts = len(path[0])
n_segs = n_wpts - 1


##################### test n-dof : to find max reachable vel per individual path seperately #######################
Estimated_vel = traj.find_max_estimated_vel_per_ndof_path(path, v_start, v_end, abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
fig = plt.figure() 
for jt in range(0, n_jts ):   
    plt.plot( Estimated_vel[jt], label='jt_{}'.format(jt))
    plt.plot( Estimated_vel[jt], 'o')
plt.xlabel("waypoints")
plt.ylabel("velocity")
plt.legend()
plt.grid()
#plt.show()


### sample and plot according to the estimated velocity 
##### fit segment according to path + estimated_vel using the previous non_zero segment planning to fit each segment of the trajetory with corresponding velocity
POS = []
VEL = []
ACC = []
JRK = [] 
TIM = []
T_SEG= []

for jt in range(0, n_jts): 
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
    
    for seg in range(0, n_segs):
        #print "\n>> seg {}: path[seg]={}, path[seg+1]={}, Estimated_vel[seg]={}, Estimated_vel[seg+1]={}".format(seg, path[i][seg], path[i][seg+1], Estimated_vel[i][seg], Estimated_vel[i][seg+1])
        jrk_dur = traj.calculate_jerk_sign_and_duration(path[jt][seg], path[jt][seg+1], Estimated_vel[jt][seg], Estimated_vel[jt][seg+1], abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
        
        #convert duration and jrk to 1d list
        jrk_dur_arr = np.array( jrk_dur)
        print jrk_dur_arr
        print jrk_dur_arr[:,1]
        T =  jrk_dur_arr[:,1].tolist()
        J =  jrk_dur_arr[:,0].tolist()
  
        #T_seg: contains the start/end time of each segment
        T_seg.append(  sum(T) + T_seg[seg] )
        t_start= T_seg[seg]
        t= t_start
        while t < T_seg[seg+1] :
            pos, vel, acc, jrk = traj.sample_segment(t, t_start, path[jt][seg], Estimated_vel[jt][seg], J, T )    
    
            Pos.append(pos)
            Vel.append(vel) #* math.copysign(1, vel))
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



### plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4,n_jts, sharex=True)

for jt in range(0, n_jts): 
    axes[0][jt].plot( TIM[jt], POS[jt])
    axes[0][jt].plot( T_SEG[jt], path[jt], 'o')
    axes[0][jt].grid()
axes[0][0].set_ylabel('position')

for jt in range(0, n_jts ):
    axes[1][jt].plot( TIM[jt], VEL[jt])
    axes[1][jt].plot( T_SEG[jt], Estimated_vel[jt],  'o')
    axes[1][jt].grid()
axes[1][0].set_ylabel('velocity')


for jt in range(0, n_jts):
    axes[2][jt].plot( TIM[jt], ACC[jt] )
    axes[2][jt].grid()
axes[2][0].set_ylabel('acceleration')


for jt in range(0, n_jts):
    axes[3][jt].plot( TIM[jt], JRK[jt] )
    axes[3][jt].grid()
    axes[3][jt].set_xlabel('Time')
axes[3][0].set_ylabel('jerk')



plt.legend()
plt.show()









