import math


class Kinematics:
    ZERO_THRESH = 0.00000001
    PI = math.pi

    def __init__(self, parameters = [0.1625, -0.425, -0.3992, 0.1333, 0.0997, 0.0996]): 
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
            double arcsin = asin(div)

            if abs(arcsin) < ZERO_THRESH: 
                arcsin = 0.0
            if arcsin < 0.0: 
                q1[0] = arcsin + 2.0*PI
            else:
                q1[0] = arcsin
                q1[1] = PI - arcsin

        elif(fabs(B) < ZERO_THRESH) {
            double div;
            if(fabs(fabs(d4) - fabs(A)) < ZERO_THRESH)
            div = SIGN(d4)*SIGN(A);
            else
            div = d4/A;
            double arccos = acos(div);
            q1[0] = arccos;
            q1[1] = 2.0*PI - arccos;
        }
        else if(d4*d4 > R) {
            return num_sols;
        }
        else {
            double arccos = acos(d4 / sqrt(R)) ;
            double arctan = atan2(-B, A);
            double pos = arccos + arctan;
            double neg = -arccos + arctan;
            if(fabs(pos) < ZERO_THRESH)
            pos = 0.0;
            if(fabs(neg) < ZERO_THRESH)
            neg = 0.0;
            if(pos >= 0.0)
            q1[0] = pos;
            else
            q1[0] = 2.0*PI + pos;
            if(neg >= 0.0)
            q1[1] = neg; 
            else
            q1[1] = 2.0*PI + neg;
        }
        }

