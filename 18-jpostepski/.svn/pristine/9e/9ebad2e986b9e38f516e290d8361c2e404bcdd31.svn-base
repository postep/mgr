import numpy as np

L0 = 1
L1 = 1
Time = 1000
m = 2
Ic = np.matrix([[1.1, 0, 0], [0, 1, 0], [0, 0, 3]]) # Array of floats
# print(Ic)

def T01(Q):
    q0 = Q[0]
    q1 = Q[1]
    return np.matrix([
        [np.cos(q1), 0, -np.sin(q1), L0*np.cos(q0)],
        [0, 1, 0, 0],
        [np.sin(q1), 0, np.cos(q1), L0*np.sin(q0)],
        [0, 0, 0, 1]])


def J0(Q:np.matrix):
    q0 = Q[0]
    q1 = Q[1]
    return np.matrix([
        [-np.sin(q0)/2, 0],
        [0, 0],
        [np.cos(q0)/2, 0],
        [0, 0],
        [1, 1],
        [0, 0]])

def J1(Q:np.matrix):
    q0 = Q[0]
    q1 = Q[1]
    return np.matrix([
        [-L0*np.sin(q0)-np.sin(q0+q1)/4, -np.sin(q0+q1)/4],
        [0, 0],
        [L0*np.cos(q0)+L1*np.cos(q0+q1), np.cos(q0+q1)/4],
        [0, 0],
        [1, 1],
        [0, 0]])

Q = np.matrix([np.pi/4, 3*np.pi/8]).getT()
dQ = np.matrix([np.pi/10, np.pi/10]).getT()
# dX = J(Q)*dQ

# print(dX)

def COM0(Q):
    q0 = Q[0]
    return np.matrix([1/2*np.cos(q0), 0, 1/2*np.sin(q0), 1]).getT()

def COM1(Q):
    q1 = Q[1]
    return np.matrix([1/4*np.cos(q1), 0, 1/4*np.sin(q1), 1]).getT()

def generate_Mi(m, I:np.matrix):
    Mi = np.eye(6, dtype=float)*m
    for x in range(3):
        for y in range(3):
            Mi[3+x][3+y] = I[x][y]
    return Mi


Tcom = T01(Q)*COM1(Q)
R = np.matrix([2, 1, 1])
r = np.trace(R*R.getT())
I = Ic + m*((R.getT()*R)*np.eye(3) - R*R.getT())

def M(Q):
    return J0(Q).getT()*generate_Mi(m1, I1(Q))*J0(Q).getT() + J1(Q).getT()*generate_Mi(m2, I2(Q))*J1(Q).getT()
K = np.matrix([1, 1, 1])*np.eye(3)
D = np.matrix([1, 1, 1])*np.eye(3)

def q_to_x(q, r):
    return np.matrix([0, 0, 0]).getT()

def x_to_q(x, r):
    return np.matrix([0, 0, 0]).getT()

def C(q, dq):
    return np.matrix([0, 0, 0]).getT()

def G(q):
    return np.matrix([0, 0, 0]).getT()

def H(q, dq):
    return np.matrix([0, 0, 0]).getT()

def M(q):
    x = q_to_x(q)
    return np.matrix([[x[1]^2+x[2]^2, -x[0]*x[1], -x[0]*x[2]],
                      [-x[0]*x[1], x[0]^2+x[2]^2, -x[1]*x[2]],
                      [-x[0]*x[2], -x[1]*x[2], x[0]^2+x[1]^2]])


# T = K*(Qd-Q) + D*(dQd-dQ) + M(Q)*ddQd + C(Q, dQ) + G(Q) + H(Q, dQ)


# F = m*ddX
# T = I*ddQ + np.cross(dQ, I*dQ)