using Plots
## Requires Gs to exist in workspace
include("lqr_design.jl")

Gsc = 1 / (1+Gs[4,1])
controller = pid(1,0,1)
Gso = Gs[4,1]*controller
Gsc = 1 / (1+Gso)
display(bodeplot(Gso))
display(rlocusplot(Gsc))
y,t,x = impulse(Gsc)
display(plot(t,y[:]))
