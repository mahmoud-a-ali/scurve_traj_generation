#!/usr/bin/env python
'''
example to check the joint motion synchronization algorithm [full trajectory] 
'''
import numpy as np
import math
import traj
from matplotlib import pyplot as plt
import rospy

rospy.init_node('traj_synchronization', log_level=rospy.DEBUG)
# limits, option_1: same limits that Jon used in first demo file
abs_max_pos = np.deg2rad(np.array([ 185.0,    60.0,  132.0,  360.0,  125.0,   360.0]))
abs_max_vel = np.deg2rad(np.array([ 150.0,   150.0,  200.0,  300.0,  300.0,   600.0]))
abs_max_acc = np.deg2rad(np.array([ 500.0,   500.0,  700.0, 1100.0, 1100.0,  2500.0]))
abs_max_jrk = np.deg2rad(np.array([ 4500.0, 4500.0, 5000.0, 8000.0, 8000.0, 16000.0]))

# limits, option_2: r-2000ic/165f that Gijs send
abs_max_pos = np.deg2rad(np.array([ 185.0, 60.0, 132.0, 360.0, 125.0, 360.0]))
abs_max_vel = np.deg2rad(np.array([ 1.300e+02, 1.150e+02, 1.250e+02, 1.800e+02, 1.800e+02,  2.600e+02 ]))
abs_max_acc = np.deg2rad(np.array([ 2.532467e+02, 2.240260e+02, 2.435065e+02, 3.506494e+02, 3.506494e+02, 5.064935e+02]))
abs_max_jrk = np.deg2rad(np.array([ 9.866757e+02, 8.728286e+02, 9.487267e+02, 1.366166e+03, 1.366166e+03,  1.973351e+03]))

#limits, option_3: M-20iB/25C that Gijs send
abs_max_pos = np.array([ 2.967060,  2.443461,  5.215218,  3.490659,  2.530727,  4.712389 ])
abs_min_pos = np.array([-2.967060, -1.745329, -2.600541, -3.490659, -2.530727, -4.712389 ])
# for now we consider even max/min position limits 
pos_limits = [min(a, abs(b)) for a,b in zip(abs_max_pos, abs_min_pos)] 
abs_max_pos = np.array(pos_limits)
abs_max_vel = np.array([ 3.577925,   3.577925,   4.537856,  7.243116,    7.243116,   15.358897])
abs_max_acc = np.array([ 12.423351,  12.423351,  15.756445,  25.149706,  25.149706,  53.329513])
abs_max_jrk = np.array([ 86.273266,  86.273266,  109.419752, 174.650735, 174.650735, 370.343857])

# print the limits
rospy.logdebug("> abs_max_pos:{}".format(abs_max_pos))
rospy.logdebug("> abs_max_vel:{}".format(abs_max_vel))
rospy.logdebug("> abs_max_acc:{}".format(abs_max_acc))
rospy.logdebug("> abs_max_jrk:{}".format(abs_max_jrk))

# path_option_1: Jon's path [the one Jon used for the first demo with zeros velocities]
path =[]
path.append([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
path.append([ 1.5, 0.7, 0.3, 0.0, 0.0, 0.0])
path.append([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
path.append([-1.5, 0.7, 0.3, 0.0, 0.0, 0.0])
path.append([ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
#there is only one point that is not changing the motion direction 
estimated_vel = [ ]
estimated_vel.append([  0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
estimated_vel.append([  0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
estimated_vel.append([ -2.7, 0.0, 0.0, 0.0, 0.0, 0.0])
estimated_vel.append([  0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
estimated_vel.append([  0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

#path_option_1: random traj & random velocities
path =[]
path.append([ 0.0,   0.0,   0.0,   0.0,   0.0, 0.0])
path.append([ 1.0,   0.4,   0.5,   0.5,   0.0, 0.0])
path.append([ 1.5,   0.2,   0.7,   0.8,   0.0, 0.0])
path.append([ 2.0,   0.0,   0.9,   1.2,   0.0, 0.0])
path.append([ 0.5,   -0.6,  0.4,   -.5,   0.0, 0.0])
path.append([ 0.0,   -0.8,  0.0,   -1.0,  0.0, 0.0])
estimated_vel = [ ]
estimated_vel.append([ 0.0,   0.0,   0.0,    0.0,   0.0,   0.0])
estimated_vel.append([ 1.4,   0.0,   0.5,    0.7,   0.0,   0.0])
estimated_vel.append([ 1.4,  -0.6,   0.5,    0.7,   0.0,   0.0])
estimated_vel.append([ 0.0,   0.0,   0.0,    0.0,   0.0,   0.0])
estimated_vel.append([-0.9,  -0.3,  -0.6,   -0.9,   0.0,   0.0])
estimated_vel.append([ 0.0,   0.0,   0.0,    0.0,   0.0,   0.0])

n_jts = len(path[0])
n_wpts = len(path)
n_segs = n_wpts - 1
min_sync_time_seg = [ ]
phases_dur_seg_jt = [ ]
phases_jrk_seg_jt = [ ]

# variables for sampling times and plotting
frq = 125.0
t_start = 0.0
abs_t = t_start
traj_pos = [ [] for jt in range(n_jts)]
traj_vel = [ [] for jt in range(n_jts)]
traj_acc = [ [] for jt in range(n_jts)]
traj_jrk = [ [] for jt in range(n_jts)]
traj_time = [ ]

waypt_times = [ ]
waypt_times.append(t_start)
for seg in range(n_segs):
	rospy.logdebug("\n\n>> seg_numer: {}".format(seg))
	min_sync_time, phase_dur_jt, phase_jrk_jt = traj.segment_synchronization(
														path[seg], path[seg+1], estimated_vel[seg], estimated_vel[seg+1],
		                        						abs_max_pos, abs_max_vel, abs_max_acc, abs_max_jrk)
	waypt_times.append(waypt_times[-1] + min_sync_time)
	while abs_t <= waypt_times[-1]:
		for jt in range(n_jts):
			p_start = path[seg][jt]
			v_start = estimated_vel[seg][jt]
			phases_dur = phase_dur_jt[jt]
			phases_jrk = phase_jrk_jt[jt]
			pos, vel, acc, jrk = traj.sample_segment(abs_t, waypt_times[-2], p_start, v_start, phases_jrk, phases_dur)
			traj_pos[jt].append(pos)
			traj_vel[jt].append(vel)
			traj_acc[jt].append(acc)
			traj_jrk[jt].append(jrk)	
		traj_time.append(abs_t)
		abs_t = abs_t + 1/frq

# plot pos, vel, acc, jrk. plot waypoints and estimated velocity as well to check if there is any difference 
fig, axes = plt.subplots(4, sharex=True)
for jt in range(0, n_jts): 
    axes[0].plot(traj_time, traj_pos[jt])
    axes[1].plot(traj_time, traj_vel[jt])
    axes[2].plot(traj_time, traj_acc[jt])
    axes[3].plot(traj_time, traj_jrk[jt])
    axes[0].plot(waypt_times, [path[wpt][jt] for wpt in range(n_wpts)], '*')
    axes[1].plot(waypt_times, [estimated_vel[wpt][jt] for wpt in range(n_wpts)], '*')
axes[0].grid()
axes[1].grid()
axes[2].grid()
axes[3].grid()
axes[0].set_ylabel('position')
axes[1].set_ylabel('velocity')
axes[2].set_ylabel('acceleration')
axes[3].set_ylabel('jerk')
axes[3].set_xlabel('Time')
plt.legend()
plt.show()

# store outputs [pos, vel, acc, jrk] in csv file
# traj_pos = list(map(list, zip(*traj_pos)))
# traj_vel = list(map(list, zip(*traj_vel)))
# traj_acc = list(map(list, zip(*traj_acc)))
# traj_jrk = list(map(list, zip(*traj_jrk)))
# import csv
# with open("sampled_traj_time_pos_vel_acc_jrk_125.csv", "wb") as csv_file:
#         writer = csv.writer(csv_file, delimiter=',')
#         for pt in range(len(traj_time)):
#             writer.writerow([traj_time[pt]] + traj_pos[pt] + traj_vel[pt] + traj_acc[pt] + traj_jrk[pt])

# with open("sampled_traj_time_positions_125.csv", "wb") as csv_file:
#         writer = csv.writer(csv_file, delimiter=',')
#         for pt in range(len(traj_time)):
#             writer.writerow([traj_time[pt]] + traj_pos[pt])
