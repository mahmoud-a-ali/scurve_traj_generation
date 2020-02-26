import nose
import numpy as np
import traj

        
        
def check_fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max):
    position, velocity, jerk, acceleration = traj.fit_traj_segment(p_start, p_end, v_start, v_end, p_max, v_max, a_max, j_max)
    
    t_start = position.boundaries[0]
    t_end = position.boundaries[-1]
    p_start_computed = position(t_start)
    v_start_computed = velocity(t_start)
    p_end_computed = position(t_end)[0]
    v_end_computed = velocity(t_end)[0]
    
    print p_start_computed, p_end_computed, v_start_computed, v_end_computed
    print " ", p_start, p_end, v_start, v_end
    
    assert np.isclose(p_start, p_start_computed)
    assert np.isclose(p_end, p_end_computed)    
    assert np.isclose(v_start, v_start_computed)
    assert np.isclose(v_end, v_end_computed)
    

#limits
p_max=30
v_max=3.0
a_max=4.0
j_max=10.0 
     
#case Debug 
def test_debug_Case():
    check_fit_traj_segment(0, 2.2360679775,      0.0, 0.14,   30.0, 0.0585401227587, 0.0780534970116, 0.195133742529 )  
    ## should output error that it violates v_max    
           



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
   



##############################################################################################
############### complex motion: v_start*v_end <0, +ve and -ve parts ##########################
##############################################################################################


############ CASE X: positive dominant motion: pos_diff > 0
### starting from +ve to -ve
def test_X1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 10.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_X2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, 5.0,      -1.5,  1.0,     p_max, v_max, a_max, j_max)   


#############CASE Y: negative dominant motion: pos_diff < 0
### starting from +ve to -ve
def test_Y1_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -10.0,      1.5, -1.0,     p_max, v_max, a_max, j_max)   
### starting from -ve to +ve
def test_Y2_complex_positive_dominant_motion():
    check_fit_traj_segment(0.0, -5.0,      -1.5,  1.0,     p_max, v_max, a_max, j_max)   



  
