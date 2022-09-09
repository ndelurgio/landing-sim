function update_mainEngine!(x,parm,t)
    Mc = get_controlCmd!(x,parm,t)
    mainEngProps = parm.mainEngine
    x.mainEngine.throttle = 0.5
    thrust_est = x.mainEngine.throttle * mainEngProps.ṁ_max * mainEngProps.isp * 9.81
    r = mainEngProps.p - x.cg
    M_mag = sqrt(Mc[1]^2+Mc[2]^2)
    x.mainEngine.uₚ = min(M_mag / (norm(r) * thrust_est),mainEngProps.uₚ_max)
    x.mainEngine.uᵣ = atan(Mc[2],Mc[1]) + pi/2
end

function update_actuators!(x, parm, t)
    update_mainEngine!(x, parm, t)
end

function get_momentCmd()

end

function get_forceCmd()

end

function get_controlCmd!(x, parm, t)
    ωᵇ = x.ωᵇ
    q = x.qⁱᵇ[2:4]
    Mc = -L * vcat(ωᵇ,q)
    x.control.Mcmd = Mc
    return Mc
end