using ControlSystems

A = [0]
B = [1]
C = [1]
D = [0]

Q = I
R = 0.5*I

K = lqr(A,B,Q,R)