import math


class Kinematics():
    ZERO_THRESH = 0.00000001
    PI = math.pi

    def __init__(self, parameters): 
        '''
        Initializes with the parameters for the robot - the default is for the ur5e
        '''
        d1 = parameters[0]
        a2 = parameters[1]
        a3 = parameters[2]
        d4 = parameters[3]
        d5 = parameters[4]
        d6 = parameters[5]

    def SIGN(x):
      return (x > 0) - (x < 0)

    def forward(input_angles):
        '''
        Takes in the 6 current joint angles of the robot arm and returns
        the end effector position in Cartesian coordinates. 
        '''
        s1 = math.sin(input_angles[0])
        c1 = math.cos(input_angles[0])
        q23 = input_angles[1]
        q234 = input_angles[1] 
        s2 = math.sin(input_angles[1])
        c2 = math.cos(input_angles[1])
        s3 = math.sin(input_angles[2])
        c3 = math.cos(input_angles[2])
        q23 += input_angles[2]
        q234 += input_angles[2]
        s4 = math.sin(input_angles[3])
        c4 = math.cos(input_angles[3])
        q234 += input_angles[3]
        s5 = math.sin(input_angles[4])
        c5 = math.cos(input_angles[4])
        s6 = math.sin(input_angles[5])
        c6 = math.cos(input_angles[5])
        s23 = math.sin(q23)
        c23 = math.cos(q23)
        s234 = math.sin(q234)
        c234 = math.cos(q234)

        result = []
        result.append(c234*c1*s5 - c5*s1)
        result.append(c6*(s1*s5 + c234*c1*c5) - s234*c1*s6)
        result.append(-s6*(s1*s5 + c234*c1*c5) - s234*c1*c6)
        result.append(6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1)
        result.append(c1*c5 + c234*s1*s5)
        result.append(-c6*(c1*s5 - c234*c5*s1) - s234*s1*s6)
        result.append(s6*(c1*s5 - c234*c5*s1) - s234*c6*s1)
        result.append(d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1)
        result.append(-s234*s5)
        result.append(-c234*s6 - s234*c5*c6)
        result.append(s234*c5*s6 - c234*c6)
        result.append(d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4))
        # not sure what the part below is for - probably related to some matrix? makes it a square
        # which could be easier for something
        result.append(0.0)
        result.append(0.0) 
        result.append(0.0)
        result.append(1.0)

        return result

    def inverse(cartesian_position, q6_des = 0.0):
    '''
    @param Takes in 4x4 end effector pose in row-major ordering (array size 16)
    @param q6_des designates what the q6 value should be in case of infinite solution on that joint
    @return 8x6 array of doubles returned, all angles are in [0, 2*PI)
    '''
        T02 = -cartesian_position[0]
        T00 = cartesian_position[1]
        T01 =  cartesian_position[2]
        T03 = -cartesian_position[3]
        T12 = -cartesian_position[4]; 
        T10 =  cartesian_position[5]
        T11 =  cartesian_position[6]
        T13 = -cartesian_position[7]
        T22 =  cartesian_position[8]
        T20 = -cartesian_position[9]
        T21 = -cartesian_position[10]
        T23 =  cartesian_position[11]

        # shoulder rotate joint (q1)
        double q1 = [0, 0]
        double A = d6*T12 - T13
        double B = d6*T02 - T03
        double R = A*A + B*B

        if abs(A) < ZERO_THRESH: 
            div = 0.0
            if abs(abs(d4) - abs(B)) < ZERO_THRESH: 
                div = -SIGN(d4)*SIGN(B)
            else: 
                div = -d4/B
            double arcsin = math.asin(div)

            if abs(arcsin) < ZERO_THRESH: 
                arcsin = 0.0
            if arcsin < 0.0: 
                q1[0] = arcsin + 2.0*PI
            else:
                q1[0] = arcsin
                q1[1] = PI - arcsin

        elif abs(B) < ZERO_THRESH:
            div = 0.0
            if abs(abs(d4) - abs(A)) < ZERO_THRESH:
                div = SIGN(d4)*SIGN(A)
            else:
                div = d4/A
                arccos = math.acos(div)
                q1[0] = arccos
                q1[1] = 2.0*PI - arccos
        elif d4*d4 > R:
            return 0

        else:
            arccos = math.acos(d4 / sqrt(R))
            arctan = math.atan2(-B, A)
            pos = arccos + arctan
            neg = -arccos + arctan

            if abs(pos) < ZERO_THRESH:
                pos = 0.0
            if abs(neg) < ZERO_THRESH:
                neg = 0.0
            if pos >= 0.0:
                q1[0] = pos
            else:
                q1[0] = 2.0*PI + pos
            if neg >= 0.0:
                q1[1] = neg
            else:
                q1[1] = 2.0*PI + neg

        # wrist2 joint (q5)
        q5 = [2][2]
        for i in range(2):
            numer = (T03*sin(q1[i]) - T13*cos(q1[i])-d4)
            div = 0.0
            if abs(abs(numer) - abs(d6)) < ZERO_THRESH:
                div = SIGN(numer) * SIGN(d6)
            else:
                div = numer / d6
                double arccos = math.acos(div)
                q5[i][0] = arccos
                q5[i][1] = 2.0*PI - arccos

        for i in range(2):
            for j in range(2):
                c1 = math.cos(q1[i])
                s1 = math.sin(q1[i])
                c5 = math.cos(q5[i][j])
                s5 = math.sin(q5[i][j])

                q6 = 0.0
                # wrist 3 joint (q6)
                if abs(s5) < ZERO_THRESH:
                    q6 = q6_des
                else 
                    q6 = math.atan2(SIGN(s5)*-(T01*s1 - T11*c1), SIGN(s5)*(T00*s1 - T10*c1))
                    if abs(q6) < ZERO_THRESH:
                       q6 = 0.0
                    if q6 < 0.0:
                       q6 += 2.0*PI
                

                double q2[2], q3[2], q4[2]
                # RRR joints (q2,q3,q4)
                c6 = math.cos(q6)
                s6 = math.sin(q6)
                x04x = -s5*(T02*c1 + T12*s1) - c5*(s6*(T01*c1 + T11*s1) - c6*(T00*c1 + T10*s1))
                x04y = c5*(T20*c6 - T21*s6) - T22*s5
                p13x = d5*(s6*(T00*c1 + T10*s1) + c6*(T01*c1 + T11*s1)) - d6*(T02*c1 + T12*s1) + T03*c1 + T13*s1
                p13y = T23 - d1 - d6*T22 + d5*(T21*c6 + T20*s6)

                c3 = (p13x*p13x + p13y*p13y - a2*a2 - a3*a3) / (2.0*a2*a3)
                if abs(abs(c3) - 1.0) < ZERO_THRESH:
                    c3 = SIGN(c3)
                elif fabs(c3) > 1.0:
                    #TODO NO SOLUTION
                    continue

                arccos = math.acos(c3)
                q3[0] = arccos
                q3[1] = 2.0*PI - arccos
                denom = a2*a2 + a3*a3 + 2*a2*a3*c3
                s3 = math.sin(arccos)
                double A = (a2 + a3*c3), B = a3*s3

                q2[0] = math.atan2((A*p13y - B*p13x) / denom, (A*p13x + B*p13y) / denom)
                q2[1] = math.atan2((A*p13y + B*p13x) / denom, (A*p13x - B*p13y) / denom)
                c23_0 = math.cos(q2[0]+q3[0])
                s23_0 = math.sin(q2[0]+q3[0])
                c23_1 = math.cos(q2[1]+q3[1])
                s23_1 = math.sin(q2[1]+q3[1])
                q4[0] = math.atan2(c23_0*x04y - s23_0*x04x, x04x*c23_0 + x04y*s23_0)
                q4[1] = math.atan2(c23_1*x04y - s23_1*x04x, x04x*c23_1 + x04y*s23_1)
                

                for k in range(2):
                    if abs(q2[k]) < ZERO_THRESH:
                        q2[k] = 0.0
                    elif q2[k] < 0.0:
                        q2[k] += 2.0*PI
                    if abs(q4[k]) < ZERO_THRESH:
                        q4[k] = 0.0
                    elif q4[k] < 0.0: 
                        q4[k] += 2.0*PI
                    q_sols[num_sols*6+0] = q1[i]   
                    q_sols[num_sols*6+1] = q2[k] 
                    q_sols[num_sols*6+2] = q3[k]
                    q_sols[num_sols*6+3] = q4[k]
                    q_sols[num_sols*6+4] = q5[i][j]
                    q_sols[num_sols*6+5] = q6

        return q_sols 

