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



max_velocities =    [  3.0,    3.0,]
max_accelerations = [  4.0,    4.0,]
max_jerks =         [  6.0,   6.0,]










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
 
 
path = np.array([ (0.0, 0.0), (1.0, 0.50) ])#,   (1.7, 1.7)])# , (2.0, 2.5)]) 
v_start= [ 0.0, 0.0]
v_end  = [ 0.0, 0.0]
 
path = np.array([ (1.0, 1.0), (2.0, 2.0) , (3.5, 3.8) ] ) 
v_start= [ 0.0, 0.0]
v_end  = [ 0.2, 0.1]
n_jts = len( v_start)
 
(trajectory_position_function, trajectory_velocity_function, trajectory_acceleration_function,
 trajectory_jerk_function) = traj.trajectory_for_path_v3(path, v_start, v_end,  max_velocities,
                                                      max_accelerations,
                                                      max_jerks)




print "\n\n>>> trajectory_jerk_function: \n{}".format(trajectory_jerk_function[0].functions )

if plot_trajectory:
    for jt in range(n_jts): 
        plt.figure()
        traj.plot.plot_trajectory(plt, trajectory_position_function[jt], trajectory_velocity_function[jt],
                              trajectory_acceleration_function[jt], trajectory_jerk_function[jt])

    plt.show()    
    
    
    plt.figure()
    traj.plot.plot_2d_path(plt.gca(), trajectory_position_function[jt], 100, label='trajectory points')
    # Plot the waypoints in the original path for comparison.
    plt.plot([q[0] for q in path], [q[1] for q in path], 'bx', label='original waypoints')

    plt.show()
















