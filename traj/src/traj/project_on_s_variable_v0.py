#!/usr/bin/env python
"""
Simple example that parametrizes a 2d joint-space path.
"""
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from matplotlib import pyplot as plt
import numpy as np
import rospy
from sensor_msgs.msg import JointState

import traj


def create_joint_trajectory_goal(piecewise_position_function, piecewise_velocity_function,
                                 piecewise_acceleration_function, joint_names, sample_period=0.008):
    print ">>>> create_joint_trajectory_goal"
    # Non-zero start times won't make sense to the controller
    assert piecewise_position_function.boundaries[0] == 0.0
    goal = FollowJointTrajectoryGoal()
    goal.trajectory.header.stamp = rospy.Time.now()
    goal.trajectory.header.frame_id = 'base_link'
    goal.trajectory.joint_names = joint_names

    for t in np.arange(0.0, piecewise_position_function.boundaries[-1], sample_period):
        point = JointTrajectoryPoint()
        point.time_from_start = rospy.Duration(t)
        point.positions = list(piecewise_position_function(t))
        point.velocities = list(piecewise_velocity_function(t))
        point.accelerations = list(piecewise_acceleration_function(t))
        goal.trajectory.points.append(point)
    print point.positions
    print point.velocities
    print point.accelerations
    
    return goal


def joint_state_callback(joint_state_msg):
    global initial_joint_states
    initial_joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]












initial_joint_states = None

# Joint limits for a fictional 6DoF arm.

max_velocities = np.deg2rad(np.array([
    3.0,
    6.0,
]))

max_accelerations = np.deg2rad(np.array([
    4.0,
    8.0,

]))

max_jerks = np.deg2rad(np.array([
    10.0,
    20.0,
 
]))



max_positions =[  30.0,    30.0,]
max_velocities =[  3.0,    3.0,]
max_accelerations = [  4.0,    4.0,]
max_jerks = [  6.0,    6.0,]










joint_names = ['joint_1', 'joint_2']

rospy.init_node('traj_demo')
plot_trajectory = rospy.get_param('~plot', True)



# Simple path
path = np.array([ (0.0, 0.7), (0.5, 1.0) ,   (0.8, 1.8)])
v_start= [.1, 0.2]
v_end = [ 0.2, 0.1]

path = np.array([ (0.0, 0.0), (1.0, 0.0) ,   (1.7, 0.0)]) 
v_start= [ 0.0, 0.1]
v_end  = [ 0.2, 0.0]
 
 
path = np.array([ (0.0, 0.0), (1.0, .50) ,   (1.3, 1.2), (1.9, 1.7)]) 
path = np.array([ (0.0, 0.0), (1.0, .50) ,   (1.5, 1.8), (1.9, 1.0)]) 
path = np.array([ (1.0, 1.0), (2.0, 2.0) , (3.5, 3.8) ] ) 


v_start= [ 0.0, 0.0]
v_end  = [ 0.2, 0.1]
 
#path = np.array([ (0.0, 0.0), (1.0, 0.5) ]) 
#v_start= [ 0.0, 0.1]
#v_end  = [ 0.0, 0.0]

n_jts = len( v_start)
 
 
##################### test n-dof : to find max reachable vel per individual path seperately #######################
rot_path = np.rot90(path).tolist()
rot_path.reverse()
Frwd_common_vel, Bkwd_common_vel, estimated_vel = traj.estimated_common_vel_for_npath(rot_path, v_start, v_end, max_positions[0], max_velocities[0], max_accelerations[0], max_jerks[0])
print "Frwd_max_vel: {}".format( Frwd_common_vel )
print "Bkwd_max_vel: {}".format( Bkwd_common_vel )
print "estimated_vel: {}".format( estimated_vel )


fig = plt.figure()   
plt.plot( Frwd_common_vel, 'r-', label='bk_vel')
plt.plot( Frwd_common_vel, 'or')

plt.plot( Bkwd_common_vel, '-b', label='fw_vel')
plt.plot( Bkwd_common_vel, 'bo')

plt.plot( estimated_vel, '-g', label='common_vel')
plt.plot( estimated_vel, 'go')

plt.xlabel("waypoints")
plt.ylabel("velocity")
plt.legend()
plt.grid()
plt.show()



(trajectory_position_function, trajectory_velocity_function, trajectory_acceleration_function,
 trajectory_jerk_function) = traj.trajectory_for_path_v0(path, estimated_vel, max_positions,  max_velocities, max_accelerations, max_jerks)

#print "\n\n\n>>> trajectory_jerk_function: \n{}".format(trajectory_jerk_function.functions )

if plot_trajectory:
#    for jt in range(n_jts):
    traj.plot.plot_trajectory(plt, trajectory_position_function, trajectory_velocity_function,
                          trajectory_acceleration_function, trajectory_jerk_function)

    plt.figure()
    traj.plot.plot_2d_path(plt.gca(), trajectory_position_function, 100, label='trajectory points')
    # Plot the waypoints in the original path for comparison.
    plt.plot([q[0] for q in path], [q[1] for q in path], 'bx', label='original waypoints')

    plt.show()















