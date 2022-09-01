using Plots
using Measures

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
end

plt = ()
theme(:dark)
## 3 Axis Position
plot(   plot(t,px,label="X"),
        plot(t,py,label="Y",title="Inerital Position"),
        plot(t,pz,label="Z"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm)
## 3D Position

## 3 Axis Velocity
plot(   plot(t,vx,label="Vx"),
        plot(t,vy,label="Vy",title="Inerital Velocity"),
        plot(t,vz,label="Vz"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm)
## 3 Axis Angular Velocity
plot(   plot(t,ωx,label="ωx"),
        plot(t,ωy,label="ωy",title="Body Rates"),
        plot(t,ωz,label="ωz"),
        lw=2,layout=(1,3),size=(1020,420),bottom_margin=4mm,xlabel="Time (s)",top_margin = 1mm,margin=-0.5mm)
## Attitude Quaternion