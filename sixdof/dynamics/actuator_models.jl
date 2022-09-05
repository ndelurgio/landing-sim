function update_mainEngine!(mainEngine,parm,t)
    mainEngine.throttle = 1.0
    mainEngine.uᵣ = 0.0
    mainEngine.uₚ = 0.0
end

function get_mainEngine(mainEngineState,mainEngineProperties,cg)
    ṁ = mainEngineProperties.ṁ_max * mainEngineState.throttle
    vₑ = mainEngineProperties.isp * 9.81
    Thrust = ṁ * vₑ
    gimbal = RotYZ(mainEngineState.uₚ, mainEngineState.uᵣ)
    thrust_axis = gimbal * mainEngineProperties.axis
    F_engine = Thrust * thrust_axis
    r = mainEngineProperties.p - cg
    M_engine = r × F_engine
    return F_engine, M_engine
end

function get_actuators(x,parm)
    F_engine,M_engine = get_mainEngine(x.mainEngine,parm.mainEngine,x.cg)
    Fₐ = F_engine
    Mₐ = M_engine
    return Fₐ, Mₐ
end