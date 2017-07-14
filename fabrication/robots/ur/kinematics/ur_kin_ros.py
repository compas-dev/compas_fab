'''
Created on 17.03.2014
@author: rustr

This file is the c++ translation from ROS's ur_kinematics package, found on 
https://github.com/ros-industrial/universal_robot/tree/hydro-devel/ur_kinematics
It is a combination of methods in ur_kin.cpp and definitions in ur_kin.h.
'''

from math import sin, cos, fabs, asin, acos, sqrt, atan2, pi

ZERO_THRESH = 0.00000001

def sign(x): # returns the sign of x
    # TODO: put this to utilities
    return  int(int((x) > 0 ) - int((x) < 0 ))


def forward_ros(q, ur_params):
    """
    Parameters: q, the 6 joint angles in radians 
                ur_params: UR defined parameters for the model, they are 
                different for UR3, UR5 and UR10 
    Returns:    T, a list of the 4x4 end effector pose in row-major ordering
    """
    
    d1, a2, a3, d4, d5, d6 =  ur_params
    
    s1, c1 = sin(q[0]) , cos(q[0])
    q234, s2, c2 = q[1], sin(q[1]), cos(q[1])
    s3, c3 = sin(q[2]), cos(q[2])
    q234 += q[2]
    q234 += q[3]
    s5, c5 = sin(q[4]), cos(q[4])
    s6, c6 = sin(q[5]), cos(q[5])
    s234, c234 = sin(q234), cos(q234)
    
    T = [0.0 for i in range(4*4)]
    
    T[0] = ((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0
    T[1] = (c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0)    
    T[2] = (-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0))
    T[3]= ((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 -  d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 -  a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3); 
    T[4] = c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0
    T[5] = (c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0))
    T[6] = (c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0))
    T[7] = ((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3)
    T[8] = ((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0)
    T[9] = ((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6)
    T[10] = (s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0)
    T[11] = (d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234)
    T[15] = 1.0
    return T


def inverse_ros(T, params, q6_des=0.0):
    """
    Parameters: T, the 4x4 end effector pose in row-major ordering
                ur_params: UR defined parameters for the model, they are 
                different for UR3, UR5 and UR10 
                q6_des, an optional parameter which designates what the q6 value 
                should take, in case of an infinite solution on that joint.   
    Returns:    q_sols, an 8x6 array of doubles returned, 8 possible q joint 
                solutions, all angles should be in [0,2 * pi]
    """
    
    d1, a2, a3, d4, d5, d6 =  params
    
    q_sols = []
    
    T02 = -T[0] 
    T00 =  T[1]  
    T01 =  T[2] 
    T03 = -T[3] 
    T12 = -T[4]
    T10 =  T[5]
    T11 =  T[6] 
    T13 = -T[7] 
    T22 =  T[8] 
    T20 = -T[9]  
    T21 = -T[10]  
    T23 =  T[11] 

    # shoulder rotate joint (q1)
    # q1[2]
    q1 = [0,0]
    A = d6*T12 - T13
    B = d6*T02 - T03
    R = A*A + B*B
    if(fabs(A) < ZERO_THRESH):
        div = 0.0
        if(fabs(fabs(d4) - fabs(B)) < ZERO_THRESH):
            div = -sign(d4)*sign(B)
        else:
            div = -d4/B
        arcsin = asin(div)
        if(fabs(arcsin) < ZERO_THRESH):
            arcsin = 0.0
        if(arcsin < 0.0):
            q1[0] = arcsin + 2.0*pi
        else:
            q1[0] = arcsin
        q1[1] = pi - arcsin
        
    elif(fabs(B) < ZERO_THRESH):
        div = 0.0
        if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH):
            div = sign(d4)*sign(A)
        else:
            div = d4/A
        arccos = acos(div)
        q1[0] = arccos
        q1[1] = 2.0*pi - arccos
    
    elif(d4*d4 > R):
        return q_sols
    else:
        arccos = acos(d4 / sqrt(R))
        arctan = atan2(-B, A)
        pos = arccos + arctan
        neg = -arccos + arctan
        if(fabs(pos) < ZERO_THRESH):
            pos = 0.0
        if(fabs(neg) < ZERO_THRESH):
            neg = 0.0
        if(pos >= 0.0):
            q1[0] = pos
        else:
            q1[0] = 2.0*pi + pos
        if(neg >= 0.0):
            q1[1] = neg
        else:
            q1[1] = 2.0*pi + neg

    # wrist 2 joint (q5)
    q5 = [[0,0],[0,0]]
    for i in range(2):
        numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4)
        div = 0.0
        if(fabs(fabs(numer) - fabs(d6)) < ZERO_THRESH):
            div = sign(numer) * sign(d6)
        else:
            div = numer / d6
        arccos = acos(div)
        q5[i][0] = arccos;
        q5[i][1] = 2.0*pi - arccos
    
    for i in range(2):
        for j in range(2):
            c1 = cos(q1[i])
            s1 = sin(q1[i])
            c5 = cos(q5[i][j])
            s5 = sin(q5[i][j])
            q6 = 0.0
            
            # wrist 3 joint (q6)
            if(fabs(s5) < ZERO_THRESH):
                q6 = q6_des
            else:
                q6 = atan2(sign(s5)*-(T01*s1 - T11*c1), sign(s5)*(T00*s1 - T10*c1))
            if(fabs(q6) < ZERO_THRESH):
                q6 = 0.0
            if(q6 < 0.0):
                q6 += 2.0*pi

            # RRR joints (q2,q3,q4)
            q2, q3, q4 = [0,0], [0,0], [0,0]

            c6 = cos(q6)
            s6 = sin(q6)
            x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1))
            x04y = c5*(T20*c6 - T21*s6) - T22*s5
            p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1
            p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)
            
            c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3)
            if(fabs(fabs(c3) - 1.0) < ZERO_THRESH):
                c3 = sign(c3)
            elif(fabs(c3) > 1.0):
                # TODO NO SOLUTION
                continue
            
            arccos = acos(c3)
            q3[0] = arccos
            q3[1] = 2.0*pi - arccos
            denom = a2*a2 + a3*a3 + 2*a2*a3*c3
            s3 = sin(arccos)
            A = (a2 + a3*c3)
            B = a3*s3
            q2[0] = atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom)
            q2[1] = atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom)
            c23_0 = cos(q2[0]+q3[0])
            s23_0 = sin(q2[0]+q3[0])
            c23_1 = cos(q2[1]+q3[1])
            s23_1 = sin(q2[1]+q3[1])
            q4[0] = atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0)
            q4[1] = atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1)
            
            for k in range(2):
                if(fabs(q2[k]) < ZERO_THRESH):
                    q2[k] = 0.0
                elif(q2[k] < 0.0):
                    q2[k] += 2.0*pi
                if(fabs(q4[k]) < ZERO_THRESH):
                    q4[k] = 0.0
                elif(q4[k] < 0.0):
                    q4[k] += 2.0*pi
                q_sols.append([q1[i], q2[k], q3[k], q4[k], q5[i][j], q6])

    return q_sols


if __name__ == "__main__":
    
    pass
    
    
