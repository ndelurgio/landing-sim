using ControlSystems
using LinearAlgebra

J = [
    4.59     0.0      0.0
    0.0      4.59     0.0
    0.0      0.0      5.3
    ]

A = [
    zeros(3,3)  zeros(3,3)
    0.5*I(3)    zeros(3,3)
    ]   

B = [
    inv(J)
    zeros(3,3)
    ]

Q = I
R = 0.01*I

L = lqr(A,B,Q,R)