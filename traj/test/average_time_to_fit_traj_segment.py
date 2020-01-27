#!/usr/bin/env python
'''
to calculate the average time to fit non-zero velocity segment,
mainly the time required to calculate jerk_signs_and_duration for segment 
'''
import matplotlib.pyplot as plt
import traj
import time

def check_fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    start_time = time.time()
    traj.fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
    execution_time =  time.time() - start_time   
    return execution_time

  

#limits
p_max=30
v_max=3.0
a_max=4.0
j_max=10.0 
     
        

for i in range (0, 20):
    #here are the different 34 cases of non-zero velocity segment planning 
    times= [check_fit_traj_segment(0.0, 1.0,      0.0, 0.0,      p_max, v_max, a_max, j_max), 
            check_fit_traj_segment(0.0, 3.0,      0.0, 0.0,      p_max, v_max, a_max, j_max),           
            check_fit_traj_segment(0.0, 5.0,      0.0, 0.0,      p_max, v_max, a_max, j_max),       
            check_fit_traj_segment(0.0, 2.0,      1.0, 1.0,      p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 3.0,      2.5, 2.5,      p_max, v_max, a_max, j_max),  
            check_fit_traj_segment(0.0, 3.0,      0.5, 0.5,      p_max, v_max, a_max, j_max),         
            check_fit_traj_segment(0.0, 10.0,     0.5, 0.5,      p_max, v_max, a_max, j_max), 
            check_fit_traj_segment(0.0, 2.0,      0.5, 1.5,      p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 5.0,      2.0, 2.5,      p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 3.0,      0.5, 2.5,      p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 5.0,      0.5, 2.5,      p_max, v_max, a_max, j_max),  
            check_fit_traj_segment(0.0, 2.0,      1.5, 0.5,      p_max, v_max, a_max, j_max),  
            check_fit_traj_segment(0.0, 5.0,      2.5, 2.0,      p_max, v_max, a_max, j_max),    
            check_fit_traj_segment(0.0, 3.0,      2.5, 0.5,      p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 5.0,      2.5, 0.5,      p_max, v_max, a_max, j_max),       
            check_fit_traj_segment(0.0, -1.0,     0.0, 0.0,      p_max, v_max, a_max, j_max),     
            check_fit_traj_segment(0.0, -3.0,     0.0, 0.0,      p_max, v_max, a_max, j_max),    
            check_fit_traj_segment(0.0, -5.0,     0.0, 0.0,      p_max, v_max, a_max, j_max),     
            check_fit_traj_segment(0.0, -2.0,    -1.0, -1.0,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -3.0,    -2.5, -2.5,     p_max, v_max, a_max, j_max), 
            check_fit_traj_segment(0.0, -3.0,    -0.5, -0.5,     p_max, v_max, a_max, j_max),         
            check_fit_traj_segment(0.0, -10.0,   -0.5, -0.5,     p_max, v_max, a_max, j_max), 
            check_fit_traj_segment(0.0, -2.0,    -0.5, -1.5,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -5.0,    -2.0, -2.5,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -3.0,    -0.5, -2.5,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -5.0,    -0.5, -2.5,     p_max, v_max, a_max, j_max),  
            check_fit_traj_segment(0.0, -2.0,    -1.5, -0.5,     p_max, v_max, a_max, j_max),  
            check_fit_traj_segment(0.0, -5.0,    -2.5, -2.0,     p_max, v_max, a_max, j_max),    
            check_fit_traj_segment(0.0, -3.0,    -2.5, -0.5,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -5.0,    -2.5, -0.5,     p_max, v_max, a_max, j_max),       
            check_fit_traj_segment(0.0, 10.0,     1.5, -1.0,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, 5.0,     -1.5,  1.0,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -10.0,    1.5, -1.0,     p_max, v_max, a_max, j_max),
            check_fit_traj_segment(0.0, -5.0,    -1.5,  1.0,     p_max, v_max, a_max, j_max)  ]   
    plt.plot(times)



plt.title("execution time for non_zero segment fitting")
plt.xlabel('case number')
plt.ylabel('time')
plt.show()
















