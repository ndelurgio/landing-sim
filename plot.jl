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
uᵣ = Vector{Float64}()
uₚ = Vector{Float64}()
Mcmd_x = Vector{Float64}()
Mcmd_y = Vector{Float64}()
Mcmd_z = Vector{Float64}()
Mctrl_x = Vector{Float64}()
Mctrl_y = Vector{Float64}()
Mctrl_z = Vector{Float64}()
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
    append!(uᵣ,state.mainEngine.uᵣ)
    append!(uₚ,state.mainEngine.uₚ)
    append!(Mcmd_x,state.control.Mcmd[1])
    append!(Mcmd_y,state.control.Mcmd[2])
    append!(Mcmd_z,state.control.Mcmd[3])
    append!(Mctrl_x,state.control.Mreal[1])
    append!(Mctrl_y,state.control.Mreal[2])
    append!(Mctrl_z,state.control.Mreal[3])
end

plt = ()
theme(:dark)
## 3 Axis Position
display(plot(plot(t,px,label="X (m)"),
        plot(t,py,label="Y (m)",title="Inerital Position"),
        plot(t,pz,label="Z (m)"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## 3D Position

## 3 Axis Velocity
display(plot(plot(t,vx,label="Vx (m/s)"),
        plot(t,vy,label="Vy (m/s)",title="Inerital Velocity"),
        plot(t,vz,label="Vz (m/s)"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## 3 Axis Angular Velocity
display(plot(plot(t,ωx*180/pi,label="ωx (deg/s)"),
        plot(t,ωy*180/pi,label="ωy (deg/s)",title="Body Rates"),
        plot(t,ωz*180/pi,label="ωz (deg/s)"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
## Attitude Angles
display(plot(plot(t,ϕ*180/pi,label="ϕ (Roll, deg)"),
        plot(t,θ*180/pi,label="θ (Pitch, deg)",title="Euler Angles"),
        plot(t,ψ*180/pi,label="ψ (Yaw, deg)"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))
display(plot(t,[qw,q₁,q₂,q₃],title="Quaternion (Inerital to Body)",label=["qw" "q1" "q2" "q3"]))
display(plot(plot(t,uᵣ*180/pi,label="uᵣ (deg)",title="Gimbal Rotation Angle"),
        plot(t,uₚ*180/pi,label="uₚ (deg)",title="Gimbal Pitch Angle"),
        lw=2,layout=(1,2),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))

display(plot(plot(t,[Mcmd_x,Mctrl_x],label=["x cmd" "x ctrl"]),
        plot(t,[Mcmd_y,Mctrl_y],label=["y cmd" "y ctrl"],title="Control Moments"),
        plot(t,[Mcmd_z,Mctrl_z],label=["z cmd" "z ctrl"]),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm,reuse=false))