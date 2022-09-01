## x: state vector: [position (inertial), velocity (inertial), angular velocity (body), quaternion (inertial to body)

using ComponentArrays
using Rotations
using LinearAlgebra
using DifferentialEquations

function update_body_rate!(ω̇ᵇ, ωᵇ, parm, t)
    Iᵇ = parm.Iᵇ
    Mᵇ = parm.Mᵇ
    ω̇ᵇ .= Iᵇ \ (Mᵇ - ωᵇ × (Iᵇ*ωᵇ))
end

function update_attitude!(q̇ⁱᵇ, qⁱᵇ, parm, t)
    ωᵇ = parm.ωᵇ
    ω₁, ω₂, ω₃ = ωᵇ ./ 2
    Ω = [
        0  -ω₁ -ω₂ -ω₃
        ω₁  0   ω₃ -ω₂
        ω₂ -ω₃  0   ω₁
        ω₃  ω₂ -ω₁  0
        ]
    q̇ⁱᵇ .= Ω * qⁱᵇ
end

function update_velocity!(v̇ⁱ, vⁱ, parm, t)
    m = parm.m
    Fᵇ = parm.Fᵇ
    ωᵇ = parm.ωᵇ
    qⁱᵇ = QuatRotation(parm.qⁱᵇ)

    vᵇ = qⁱᵇ * vⁱ 
    v̇ᵇ = (1/m) * Fᵇ - ωᵇ × vᵇ
    v̇ⁱ .= qⁱᵇ \ v̇ᵇ
end

function update_position!(ṗⁱ, pⁱ, parm, t)
    vⁱ = parm.vⁱ
    ṗⁱ .= vⁱ
end

function update_state!(ẋ, x, parm, t)
    m = parm.m
    Iᵇ = parm.Iᵇ
    Mᵇ = parm.Mᵇ
    Fᵇ = parm.Fᵇ
    update_body_rate!(ẋ.ωᵇ, x.ωᵇ,  (Iᵇ=Iᵇ, Mᵇ=Mᵇ),                   t)
    update_attitude!(ẋ.qⁱᵇ, x.qⁱᵇ, (ωᵇ=x.ωᵇ,),                       t)
    update_velocity!(ẋ.vⁱ,  x.vⁱ,  (m=m, Fᵇ=Fᵇ, ωᵇ=x.ωᵇ, qⁱᵇ=x.qⁱᵇ), t)
    update_position!(ẋ.pⁱ,  x.pⁱ,  (vⁱ=x.vⁱ,),                        t)
end

## Init State
x₀ = ComponentArray(pⁱ = [0.0,0.0,0.0], vⁱ = [0.0,0.0,0.0], ωᵇ = [0.5,0.8,0.0], qⁱᵇ = [1.0,0.0,0.0,0.0])
# x = x₀

## Init State Derivative
# ẋ₀ = ComponentArray(ṗⁱ = [0.0,0.0,0.0], v̇ⁱ = [0.0,0.0,0.0], ω̇ᵇ = [0.0,0.0,0.0], q̇ⁱᵇ = [0.0,0.0,0.0,0.0])
# ẋ = ẋ₀

## Init mass & inertia tensor
m₀ = 200.0
m = m₀
I₀ = [
    100.0   10.0     0.0
    10.0     100.0   0.0
    0.0     0.0     200.0
]
Iᵇ = I₀

Mᵇ = [0.0,0.0,0.0]
Fᵇ = [0.0,0.0,0.0]
parm = (m=m,Iᵇ=Iᵇ,Mᵇ=Mᵇ,Fᵇ=Fᵇ)

tspan = (0,50)
prob = ODEProblem(update_state!,x₀,tspan,parm)
sim = solve(prob,RK4(),dt=0.01)