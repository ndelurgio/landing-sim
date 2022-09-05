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

parm = (mainEngine=mainEngine,)
prob = ODEProblem(update_state!,vehicle,tspan,parm)
reltol = 1e-8
println("Configuration Complete. Running Simulation...")
sim = solve(prob,RK4(),reltol=reltol)
println("Finished. Plotting...")
include("plot.jl")
println("Done.")