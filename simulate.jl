using ComponentArrays
using Rotations
using LinearAlgebra
using DifferentialEquations

include("config.jl") ## Create vehicle configuration

include("sixdof/dynamics/eom.jl")
include("sixdof/dynamics/enviornment.jl")
include("sixdof/dynamics/actuator_models.jl")
include("sixdof/dynamics/sensor_models.jl")
include("sixdof/gnc/control.jl")
include("sixdof/gnc/guidance.jl")


prob = ODEProblem(update_state!,vehicle,tspan,parm)
reltol = 1e-8
dt = 0.01
println("Configuration Complete. Running Simulation...")
sim = solve(prob,RK4(),adaptive=false,dt=dt)
println("Finished. Plotting...")
include("plot.jl")
println("Done.")