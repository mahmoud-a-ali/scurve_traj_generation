import nose
import numpy as np
import traj

    
### main test function to check start/end and intermediate pos, vel, acc, jrk    
def check_fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    position, velocity, acceleration, jerk = traj.fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
    
    #### 1) test if the calculated start/end pos/vel are equal to the given ones
    t_start = position.boundaries[0]
    t_end = position.boundaries[-1]
    p_start_computed = position(t_start)
    v_start_computed = velocity(t_start)
    p_end_computed = position(t_end)[0]
    v_end_computed = velocity(t_end)[0]

    assert np.isclose(p_start, p_start_computed)
    assert np.isclose(p_end, p_end_computed)    
    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(v_end, v_end_computed)
    
    #### 2) test if the calculated  pos/vel/acc/jrk [at each time_instatnt has change in segment_phase] are within the given limits p_max, v_max, a_max, j_max    
    for phase_time in position.boundaries:
        p_computed = position(phase_time)
        v_computed = velocity(phase_time)
        a_computed = acceleration(phase_time)
        j_computed = jerk(phase_time)
#        print "p_computed, v_computed,  a_computed, j_computed :"         
#        print p_computed, v_computed,  a_computed, j_computed          
        assert np.less_equal( abs(p_computed), p_max+1e-6) #added 1e-6 because sometime it gives fails even if the difference is less than (1e-6) 
        assert np.less_equal( abs(v_computed), v_max+1e-6)        
        assert np.less_equal( abs(a_computed), a_max+1e-6)
        assert np.less_equal( abs(j_computed), j_max+1e-6)




p_max=10
v_max=3.0
a_max=4.0
j_max=10.0 



############### CASE A: v_start = v_end = 0    [normal case, start and end velocities are zeros]
#case A1:  no limit is reached     
def test_not_max_vel_nor_max_acc_zero_equal_velocity():
    check_fit_traj_segment(0.0, 1.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A2:  acc_limit is reached     
def test_max_acc_but_not_max_vel_zero_equal_velocity():
    check_fit_traj_segment(0.0, 3.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A3:  vel_limit and acc_limit are reached    
def test_max_vel_and_max_acc_zero_equal_velocity():
    check_fit_traj_segment(0.0, 5.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)        
    

############## CASE B: v_start = v_end !=0   [ start and end velocities are equal but not null]
#case B1:  no limit is reached
def test_not_max_vel_nor_max_acc_nonzero_equal_veolcity():
    check_fit_traj_segment(0.0, 2.0,      1.0, 1.0,     p_max, v_max, a_max, j_max)
       
#case B2:  vel_limit is reached 
def test_max_vel_but_not_max_acc_nonzero_equal_veolcity():
    check_fit_traj_segment(0.0, 3.0,      2.5, 2.5,     p_max, v_max, a_max, j_max)   
   
#case B3:  acc_limit is reached
def test_max_acc_but_not_max_vel_nonzero_equal_veolcity():
    check_fit_traj_segment(0.0, 3.0,      0.5, 0.5,     p_max, v_max, a_max, j_max)           
   
#case B4:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_nonzero_equal_veolcity():
    check_fit_traj_segment(0.0, 10.0,     0.5, 0.5,     p_max, v_max, a_max, j_max)    
   


############## CASE C:  v_start < v_end [acceleration]
#case C1:  no limit is reached
def test_not_max_vel_nor_max_acc_acceleration():
    check_fit_traj_segment(0.0, 2.0,       0.5, 1.5,    p_max, v_max, a_max, j_max)    
    
##case C2:  vel_limit is reached 
def test_max_vel_but_not_max_acc_acceleration():
    check_fit_traj_segment(0.0, 5.0,      2.0, 2.5,     p_max, v_max, a_max, j_max)    
    
##case C3:  acc_limit is reached
def test_max_acc_but_not_max_vel_acceleration():
    check_fit_traj_segment(0.0, 3.0,      0.5, 2.5,     p_max, v_max, a_max, j_max) 
    
##case C4:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_acceleration():
    check_fit_traj_segment(0.0, 5.0,       0.5, 2.5,    p_max, v_max, a_max, j_max)    
   
   
   
############## CASE D:  v_start > v_end [deceleration]
#case D1:  no limit is reached
def test_not_max_vel_nor_max_acc_deceleration():
    check_fit_traj_segment(0.0, 2.0,       1.5, 0.5,    p_max, v_max, a_max, j_max)     
    
##case D2:  vel_limit is reached 
def test_max_vel_but_not_max_acc_deceleration():
    check_fit_traj_segment(0.0, 5.0,       2.5, 2.0,    p_max, v_max, a_max, j_max)        
    
##case D3:  acc_limit is reached
def test_max_acc_but_not_max_vel_deceleration():
    check_fit_traj_segment(0.0, 3.0,       2.5, 0.5,    p_max, v_max, a_max, j_max)
    
##case D4:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_deceleration():
    check_fit_traj_segment(0.0, 5.0,       2.5, 0.5,    p_max, v_max, a_max, j_max)        
    
  


############### CASE A-: v_start = v_end = 0    [normal case, negative motion]
#case A1-:  no limit is reached     
def test_not_max_vel_nor_max_acc_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -1.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A2-:  acc_limit is reached     
def test_max_acc_but_not_max_vel_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)      
    
#case A3-:  vel_limit and acc_limit are reached    
def test_max_vel_and_max_acc_zero_equal_velocity_negative_motion():
    check_fit_traj_segment(0.0, -5.0,      0.0, 0.0,     p_max, v_max, a_max, j_max)        
    

############## CASE B-: v_start = v_end !=0   [ start and end velocities are equal but not null]
#case B1-:  no limit is reached
def test_not_max_vel_nor_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -2.0,      -1.0, -1.0,     p_max, v_max, a_max, j_max)
       
#case B2-:  vel_limit is reached 
def test_max_vel_but_not_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,     -2.5, -2.5,     p_max, v_max, a_max, j_max)   
   
#case B3-:  acc_limit is reached
def test_max_acc_but_not_max_vel_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      -0.5, -0.5,     p_max, v_max, a_max, j_max)           
   
#case B4-:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_nonzero_equal_veolcity_negative_motion():
    check_fit_traj_segment(0.0, -10.0,     -0.5, -0.5,     p_max, v_max, a_max, j_max)    
   

############## CASE C-:  v_start < v_end [acceleration]
#case C1-:  no limit is reached
def test_not_max_vel_nor_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -2.0,       -0.5, -1.5,    p_max, v_max, a_max, j_max)    
    
##case C2-:  vel_limit is reached 
def test_max_vel_but_not_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,      -2.0, -2.5,     p_max, v_max, a_max, j_max)    
    
##case C3-:  acc_limit is reached
def test_max_acc_but_not_max_vel_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -3.0,      -0.5, -2.5,     p_max, v_max, a_max, j_max) 
    
##case C4-:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_acceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -0.5, -2.5,    p_max, v_max, a_max, j_max)    
   

############## CASE D-:  v_start > v_end [deceleration]
#case D1:  no limit is reached
def test_not_max_vel_nor_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -2.0,       -1.5, -0.5,    p_max, v_max, a_max, j_max)     
    
##case D2:  vel_limit is reached 
def test_max_vel_but_not_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -2.5, -2.0,    p_max, v_max, a_max, j_max)        
    
##case D3:  acc_limit is reached
def test_max_acc_but_not_max_vel_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -3.0,       -2.5, -0.5,    p_max, v_max, a_max, j_max)
    
##case D4:  vel_limit and acc_limit are reached   
def test_max_vel_and_max_acc_deceleration_negative_motion():
    check_fit_traj_segment(0.0, -5.0,       -2.5, -0.5,    p_max, v_max, a_max, j_max)        
   



###############################################################################################
################ complex motion: v_start*v_end <0, +ve and -ve parts ##########################
###############################################################################################


############ CASE X: positive dominant motion: pos_diff > 0
### starting from +ve to -ve
def test_X1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 8.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_X2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 5.0,      -1.5, +1.0,     p_max, v_max, a_max, j_max)   


#############CASE Y: negative dominant motion: pos_diff < 0
### starting from +ve to -ve
def test_Y1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -8.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_Y2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -5.0,      -1.5, +1.0,     p_max, v_max, a_max, j_max)   






################################################################################################
################# cases where exception should be raised by the function #######################
################################################################################################
def test_exception_vel_motion_opposite_to_pos_motion_1():
    with nose.tools.assert_raises_regexp(ValueError, "non feasible case: vel_motion opposite to pos_motion"):
        position, velocity, jerk, acceleration = traj.fit_traj_segment(0.0, 5.0,      -1.5, -1.0,     p_max, v_max, a_max, j_max)   

def test_exception_vel_motion_opposite_to_pos_motion_2():
    with nose.tools.assert_raises_regexp(ValueError, "non feasible case: vel_motion opposite to pos_motion"):
        position, velocity, jerk, acceleration = traj.fit_traj_segment(0.0, -5.0,      1.5, 1.0,     p_max, v_max, a_max, j_max)   


def test_exception_violate_min_pos_to_vf():
    with nose.tools.assert_raises_regexp(ValueError, "non feasible case: violate min_pos_to_vf"):
        position, velocity, jerk, acceleration = traj.fit_traj_segment(4.0, 4.4,      0.2, 2.5,     p_max, v_max, a_max, j_max)   

def test_exception_violate_p_max():
    with nose.tools.assert_raises_regexp(ValueError, "non feasible case: violate p_max"):
        position, velocity, jerk, acceleration = traj.fit_traj_segment(9.0, 8.0,      2.5, -3.0,     p_max, v_max, a_max, j_max)   



