using ControlSystems
using Plots

A = [0 1
     0 0]
B = [0
     1 / 4.59]
C = [1 0]
D = 0

sys = ss(A,B,C,D)
Gs = tf(sys)
display(bodeplot(Gs))
Gsc = 1 / (1 + Gs)
display(rlocusplot(Gsc))
display(step(Gsc))