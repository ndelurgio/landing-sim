using Plots
using Measures
using ReferenceFrameRotations

## Extract Telemetry
t = sim.t
px = Vector{Float64}()
py = Vector{Float64}()
pz = Vector{Float64}()
vx = Vector{Float64}()
vy = Vector{Float64}()
vz = Vector{Float64}()
ωx = Vector{Float64}()
ωy = Vector{Float64}()
ωz = Vector{Float64}()
ϕ = Vector{Float64}() # roll
θ = Vector{Float64}() # pitch
ψ = Vector{Float64}() # yaw
qw = Vector{Float64}()
q₁ = Vector{Float64}()
q₂ = Vector{Float64}()
q₃ = Vector{Float64}()
for state in sim.u
    append!(px,state.pⁱ[1])
    append!(py,state.pⁱ[2])
    append!(pz,state.pⁱ[3])
    append!(vx,state.vⁱ[1])
    append!(vy,state.vⁱ[2])
    append!(vz,state.vⁱ[3])
    append!(ωx,state.ωᵇ[1])
    append!(ωy,state.ωᵇ[2])
    append!(ωz,state.ωᵇ[3])
    # r_xyz = RotXYZ(QuatRotation(state.qⁱᵇ))
    r_xyz = quat_to_angle(Quaternion(state.qⁱᵇ),:ZYX)
    # append!(ϕ,r_xyz[1])
    # append!(θ,r_xyz[2])
    # append!(ψ,r_xyz[3])
    append!(ϕ,r_xyz.a3)
    append!(θ,r_xyz.a2)
    append!(ψ,r_xyz.a1)
    append!(qw,state.qⁱᵇ[1])
    append!(q₁,state.qⁱᵇ[2])
    append!(q₂,state.qⁱᵇ[3])
    append!(q₃,state.qⁱᵇ[4])
end

plt = ()
theme(:dark)
## 3 Axis Position
display(plot(plot(t,px,label="X"),
        plot(t,py,label="Y",title="Inerital Position"),
        plot(t,pz,label="Z"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## 3D Position

## 3 Axis Velocity
display(plot(plot(t,vx,label="Vx"),
        plot(t,vy,label="Vy",title="Inerital Velocity"),
        plot(t,vz,label="Vz"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## 3 Axis Angular Velocity
display(plot(plot(t,ωx,label="ωx"),
        plot(t,ωy,label="ωy",title="Body Rates"),
        plot(t,ωz,label="ωz"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## Attitude Angles
display(plot(plot(t,ϕ,label="ϕ (Roll)"),
        plot(t,θ,label="θ (Pitch)",title="Euler Angles"),
        plot(t,ψ,label="ψ (Yaw)"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
display(plot(t,[qw,q₁,q₂,q₃],title="Quaternion (Inerital to Body)",label=["qw" "q1" "q2" "q3"]))