function update_mainEngine!(x,parm,t)
    Mc, Fc = get_controlCmd!(x,parm,t)
    mainEngProps = parm.mainEngine
    x.mainEngine.throttle = Fc[3] / (mainEngProps.ṁ_max * mainEngProps.isp * 9.81)
    if x.mainEngine.throttle < 0.0
        x.mainEngine.throttle = 0.0
    elseif x.mainEngine.throttle > 1.0
        x.mainEngine.throttle = 1.0
    end

    thrust_est = x.mainEngine.throttle * mainEngProps.ṁ_max * mainEngProps.isp * 9.81
    r = mainEngProps.p - x.cg
    M_mag = sqrt(Mc[1]^2+Mc[2]^2)
    x.mainEngine.uₚ = min(M_mag / (norm(r) * thrust_est),mainEngProps.uₚ_max)
    x.mainEngine.uᵣ = atan(Mc[2],Mc[1]) + pi/2
end

function update_actuators!(x, parm, t)
    update_mainEngine!(x, parm, t)
end

function get_controlCmd!(x, parm, t)
    ## attitude controller
    ωᵇ = x.ωᵇ
    q = x.qⁱᵇ[2:4]
    Mc = -L * vcat(ωᵇ,q)

    ## descent velocity controller
    if x.pⁱ[3] < 25
        v_cmd = -0.5
    else
        v_cmd = -10
    end

    v_curr = x.vⁱ[3]
    v_err = v_cmd - v_curr
    K = 1
    a_cmd = v_err*K
    Fc = [0.0,0.0,(a_cmd+9.81)*x.m] ## feed forward gravity
    x.control.Mcmd = Mc
    x.control.Fcmd = Fc
    return Mc, Fc
end