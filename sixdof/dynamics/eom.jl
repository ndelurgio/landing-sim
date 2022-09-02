## x: state vector: [position (inertial), velocity (inertial), angular velocity (body), quaternion (inertial to body)

function update_body_rate!(ω̇ᵇ, ωᵇ, parm, t)
    Iᵇ = parm.Iᵇ
    Mᵇ = parm.Mᵇ
    ω̇ᵇ .= Iᵇ \ (Mᵇ - ωᵇ × (Iᵇ*ωᵇ))
end

function update_attitude!(q̇ⁱᵇ, qⁱᵇ, parm, t)
    ωᵇ = parm.ωᵇ
    ω₁, ω₂, ω₃ = ωᵇ
    Ω = [
        0  -ω₁ -ω₂ -ω₃
        ω₁  0   ω₃ -ω₂
        ω₂ -ω₃  0   ω₁
        ω₃  ω₂ -ω₁  0
        ]
    q̇ⁱᵇ .= 0.5 * Ω * qⁱᵇ
end

function update_velocity!(v̇ⁱ, vⁱ, parm, t)
    m = parm.m
    Fᵇ = parm.Fᵇ
    ωᵇ = parm.ωᵇ
    qⁱᵇ = QuatRotation(parm.qⁱᵇ)

    vᵇ = qⁱᵇ * vⁱ 
    v̇ᵇ = (1/m) * Fᵇ + ωᵇ × vᵇ
    v̇ⁱ .= qⁱᵇ \ v̇ᵇ
end

function update_position!(ṗⁱ, pⁱ, parm, t)
    vⁱ = parm.vⁱ
    ṗⁱ .= vⁱ
end

function update_mass!(ṁ, m, t)
    ṁ = 0.0
end

function update_inertia!(İᵇ, Iᵇ, t)
    İᵇ = [
        0 0 0
        0 0 0
        0 0 0
    ]
end

function update_state!(ẋ, x, parm, t)
    Fₑ, Mₑ = get_enviornment(x)
    Fₐ, Mₐ = get_actuators(x)
    Fᵇ = Fₑ .+ Fₐ
    Mᵇ = Mₑ .+ Mₐ

    update_mass!(ẋ.m,x.m,t)
    update_inertia!(ẋ.Iᵇ, x.Iᵇ, t)
    update_body_rate!(ẋ.ωᵇ, x.ωᵇ,  (Iᵇ=x.Iᵇ, Mᵇ=Mᵇ),                   t)
    update_attitude!(ẋ.qⁱᵇ, x.qⁱᵇ, (ωᵇ=x.ωᵇ,),                       t)
    update_velocity!(ẋ.vⁱ,  x.vⁱ,  (m=x.m, Fᵇ=Fᵇ, ωᵇ=x.ωᵇ, qⁱᵇ=x.qⁱᵇ), t)
    update_position!(ẋ.pⁱ,  x.pⁱ,  (vⁱ=x.vⁱ,),                       t)
end