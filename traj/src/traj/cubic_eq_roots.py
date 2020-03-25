#!/usr/bin/env python
import math


def real_roots_cubic_eq ( a,  b,  c,  d):
    '''
    This function finds the  real roots of a cubic equation, 
    '''
    # print "real_roots_cubic_eq, eq_info: a={} b={} c={} d={} ".format(a,  b, c, d)   
    rt1=0
    rt2=0
    rt3=0
    if (a == 0.0000000):
        # print "The coefficient of the cube of x is 0. Please use the utility for a SECOND degree quadratic. No further action taken."
        rt1, rt2, n_rts = quad_eq_real_root ( b, c, d)
        return rt1, rt2, -100, n_rts
    
    if ( abs(d) < 1e-20):
        # print "One root is 0. Now divide through by x and use the utility for a SECOND degree quadratic to solve the resulting equation for the other two roots. No further action taken."
        rt1 = 0
        rt2, rt3, n_rts = quad_eq_real_root (a, b, c)
        return rt1, rt2, rt3, n_rts+1
    
    b /= a
    c /= a
    d /= a

    q = (3.0*c - (b*b))/9.0
    r = -(27.0*d) + b*(9.0*c - 2.0*(b*b))
    r /= 54.0
    disc = q*q*q + r*r
    # print "disc= {}".format(disc)
    term1 = (b/3.0)
    if (disc > 0.0): # one root real, two are complex
        s = r + math.sqrt(disc)
        if s<0:
            mins_s=-s
            s=  - mins_s**(1.0/3.0) 
        else:
            s= s**(1.0/3.0)
            
        t = r - math.sqrt(disc)  
        if t<0:
            mins_t=-t
            t=  - mins_t**(1.0/3.0) 
        else:
            t= t**(1.0/3.0)
            
        x1r= -term1 + s + t
        term1 += (s + t)/2.0
        x3r = -term1
        x2r = -term1
        term1 = math.sqrt(3.0)*(-t + s)/2
        x2r = term1
        rt1 = x1r
        rt2 = -100.0
        rt3 = -100.0
        return rt1, rt2, rt3, 1
        # End if (disc > 0)
    
    #The remaining options are all real
    if disc==0.000 : # All roots real, at least two are equal.
        disc=0   
        if r<0:
            mins_r=-r
            r13=  - mins_r**(1.0/3.0) 
        else:
            r13= r**(1.0/3.0)
            
        x1r= -term1 + 2.0*r13
        x3r = -(r13 + term1)
        x2r = -(r13 + term1)
        rt1= x1r
        rt2 = x2r
        rt3= x3r
        return rt1, rt2, rt3, 2
        #End if (disc == 0)
    
    #Only one option left is that all roots are real and unequal (to get here, q < 0)
    # print "last case: disc < 0 , disc= "
    # print disc
    else:
        q = -q
        dum1 = q*q*q
        dum1 = math.acos(r/math.sqrt(dum1))
        r13 = 2.0*math.sqrt(q)
        x1r= -term1 + r13*math.cos(dum1/3.0)
        x2r = -term1 + r13*math.cos((dum1 + 2.0*math.pi)/3.0)
        x3r = -term1 + r13*math.cos((dum1 + 4.0*math.pi)/3.0)
        rt1= x1r
        rt2= x2r
        rt3= x3r
        return rt1, rt2, rt3, 3
        #End of cubicSolve





def quad_eq_real_root (a, b, c):
    '''
    This function finds the  real roots of a quadratic equation, 
    '''
    disc= b*b - 4*a*c
    if(a==0.000000):
        if(b==0.000000):
            rt1= 0
            rt2= 0
            return rt1, rt2, 0
        else :
            rt1= -c/b
            rt2= -c/b
            return rt1, rt2, 1
        
    else :
        if(disc > 0):
            rt1 = (-b - math.sqrt(disc) ) / (2*a)
            rt2 = (-b + math.sqrt(disc) ) / (2*a)
            return rt1, rt2, 2
        else :
            rt1= 0
            rt2= 0
            return rt1, rt2, 0




def min_positive_root2( r1,  r2):    
    '''
    This function finds the minimum positive number of two numbers 
    '''
    min_rt = -1
    if (r1>0 and r2>0 ):
        if r1<r2:
            min_rt=r1
        else:
            min_rt=r2
    elif (r1>0 ):
        min_rt= r1
    elif (r2>0 ):
        min_rt= r2
    else:
        raise ValueError("there is no real positive roots!" ) 
    return min_rt


def min_positive_root3( r1,  r2,  r3):
    '''
    This function finds the minimum positive number of three numbers
    '''
    min_rt=-1
    if (r1>0 and r2>0 and r3>0):
        if r1<r2:
            min_rt=min_positive_root2( r1,  r3)
        else:
            min_rt=min_positive_root2( r2,  r3)
    elif (r1>0 and r2>0):
        min_rt=min_positive_root2( r1,  r2)
    elif (r1>0 and r3>0):
        min_rt=min_positive_root2( r1,  r3)
    elif (r2>0 and r3>0):
        min_rt=min_positive_root2( r2,  r3)
    elif (r1>0):
        min_rt= r1
    elif (r2>0):
        min_rt= r2
    elif (r3>0 ):
        min_rt= r3
    else:
        raise ValueError("there is no real positive roots!" ) 
    return min_rt












