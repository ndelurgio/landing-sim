function update_mainEngine!(mainEngine,parm,t)
    mainEngine.throttle = 1.0
    mainEngine.uᵣ = 0.0
    mainEngine.uₚ = 0.0
end

function update_actuators!(x, parm, t)
    update_mainEngine!(x.mainEngine, parm, t)
end

function get_momentCmd()

end

function get_forceCmd()

end

function get_controlCmd(x, parm)
    ωᵇ = x.ωᵇ
    q = x.qⁱᵇ[2:4]
    Mc = -L * vcat(ωᵇ,q)
    return Mc
end