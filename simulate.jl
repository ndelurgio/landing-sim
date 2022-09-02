using ComponentArrays
using Rotations
using LinearAlgebra
using DifferentialEquations

include("config.jl") ## Create vehicle configuration

include("sixdof/dynamics/eom.jl")
include("sixdof/dynamics/enviornment.jl")
include("sixdof/dynamics/actuator_models.jl")
include("sixdof/dynamics/sensor_models.jl")
parm = ()
prob = ODEProblem(update_state!,vehicle,tspan,parm)
# print(prob)s
reltol = 1e-8
sim = solve(prob,RK4(),reltol=reltol)

include("plot.jl")