import nose
import traj
import numpy as np
'''
to test cubic_eq_roos.py, main objective to get the min positive real root for cubic equation, 
if non positive real roots exist, it should raise error
'''
        
        
def check_min_positive_real_root_for_cubic_eq(a, b, c, d, root_value):
    r1, r2, r3, n_rts= traj.real_roots_cubic_eq ( a,  b,  c,  d)
    if n_rts==1:
        r = r1
    elif n_rts ==2:
        r = traj.min_positive_root2(r1, r2)
    elif n_rts ==3:
        r = traj.min_positive_root3(r1, r2, r3)
     
    print r1, r2, r3, n_rts     
    assert np.isclose(r, root_value)
 
    
      
def test_3_different_positive_real_roots():
    check_min_positive_real_root_for_cubic_eq(1, -6, 11, -6, 1)
    
    
def test_two_positive_real_root():
    check_min_positive_real_root_for_cubic_eq(0, 1, -3, 2, 1)
    
    
def test_two_positive_real_root_2():
    check_min_positive_real_root_for_cubic_eq(1, -3, 2, 0, 1)
        
    
def test_one_positive_real_root():
    check_min_positive_real_root_for_cubic_eq(1, 1, 1, -3, 1)
    


### when error should be raised 
def test_3_different_negative_real_roots():
    with nose.tools.assert_raises_regexp(ValueError, "there is no real positive roots!"):
        check_min_positive_real_root_for_cubic_eq(1, 6, 11, 6,  0)
        
