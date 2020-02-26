#!/usr/bin/env python
import math
import traj 
from matplotlib import pyplot as plt
import rospy
import numpy as np
import time


def find_jrk_for_T_and_pos(T, pos_diff):
    print "find jrk"
    
def calculate_pos_diff_for_T_and_jrk(tj1, ta1, tj, ta, tv, jk, v0):
    pos_diff= jk*ta**2*tj + 2*jk*ta*ta1*tj1 + 3*jk*ta*tj**2 + jk*tv*ta*tj + 2*jk*ta*tj1**2 + 2*v0*ta + (jk*ta1**2*tj1)/2 + 4*jk*ta1*tj*tj1 + (3*jk*ta1*tj1**2)/2 + jk*tv*ta1*tj1 + v0*ta1 + 2*jk*tj**3 + jk*tv*tj**2 + 4*jk*tj*tj1**2 + 4*v0*tj + jk*tj1**3 + jk*tv*tj1**2 + 2*v0*tj1 + tv*v0
    return pos_diff
    
    
def calculate_pos_diff_for_T_and_jrk_6phs( tj, ta, jk, v0):
    pos_diff= jk*ta**2*tj + 3*jk*ta*tj**2 + 2*v0*ta + 2*jk*tj**3 + 4*v0*tj
    return pos_diff
    
    
def calculate_inflection_vel_acc(tj1, ta1, tj, ta, tv, jk, v0):
    a1 =  jk*tj1     ;
    a2 =           a1;
    a3 = -jk*tj1 + a2;  
    
    a4 =  jk*tj  + a3;
    a5 =           a4;
    a6 = -jk*tj  + a5; 
    a7 =           a6; 
    a8 = -jk*tj +  a7;
    a9 =           a8; 
    a10=  jk*tj +  a9; 
     
    
    
    v1 =  jk*tj1*tj1/2 +           + v0;
    v2 =                  a1*ta1   + v1;
    v3 = -jk*tj1*tj1/2 +  a2*tj1   + v2;
    
    v4 =  jk*tj*tj/2 +    a3*tj    + v3;
    v5 =                  a4*ta    + v4;
    v6 = -jk*tj*tj/2 +    a5*tj    + v5;
    v7 =                  a6*tv    + v6;
    v8 = -jk*tj*tj/2 +    a7*tj    + v7;
    v9 =                  a8*ta    + v8;
    v10=  jk*tj*tj/2 +    a9*tj    + v9;
    
    acc_vec = [0.0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10]
    vel_vec = [v0 , v1, v2, v3, v4, v5, v6, v7, v8, v9, v10]
    return acc_vec, vel_vec
    
    
def calculate_jrk_for_T(tj1, ta1, tj, ta, tv, v0, pos_diff): 
    jrk =-(2*ta*v0 - pos_diff + ta1*v0 + 4*tj*v0 + 2*tj1*v0 + tv*v0)/(ta**2*tj + 2*ta*ta1*tj1 + 3*ta*tj**2 + tv*ta*tj + 2*ta*tj1**2 + (ta1**2*tj1)/2 + 4*ta1*tj*tj1 + (3*ta1*tj1**2)/2 + tv*ta1*tj1 + 2*tj**3 + tv*tj**2 + 4*tj*tj1**2 + tj1**3 + tv*tj1**2)
    return jrk    
    
    
    
    
    
    
    
    
    
########### test case to check functionality    
#tj1=0.2
#ta1=0.0 
#tj=0.3 
#ta=0.2 
#tv=0.0 
#jk=10.0 
#v0=0.0
#
#pd = calculate_pos_diff_for_T_and_jrk(tj1, ta1, tj, ta, tv, jk, v0) 
#pd6 = calculate_pos_diff_for_T_and_jrk_6phs( tj, ta, jk, v0)
#print "pd={} , pd6={}".format( pd, pd6)
#
#jrk = calculate_jrk_for_T(tj1, ta1, tj, ta, tv, v0, pd/2.0)
#print "pd={} ".format( jrk )
#
#
#pd = calculate_pos_diff_for_T_and_jrk(tj1, ta1, tj, ta, tv, jrk, v0) 
#print "pd={}".format(pd)
#
#vel_vec, acc_vec = calculate_inflection_vel_acc(tj1, ta1, tj, ta, tv, jk, v0)
#print vel_vec
#print acc_vec
# 
 
 
 