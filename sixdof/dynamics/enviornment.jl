## Gravity
function get_gravity(m,qⁱᵇ, g=-9.8)
    Fgⁱ = [0.0, 0.0, m * g]
    Fg = qⁱᵇ * Fgⁱ ## Body Frame
    Mg = [0.0,0.0,0.0]
    return Fg, Mg
end

function get_disturbances(t)
    Fd =  2*sin(t) * ones(3)
    Md =  0.05*cos(t) * [1,1,0]
    return Fd, Md
end

function get_enviornment(x,parm,t)
    Fg, Mg = get_gravity(x.m,QuatRotation(x.qⁱᵇ))
    if parm.enable_disturbances
        Fd, Md = get_disturbances(t)
    else
        Fd = [0.0,0.0,0.0]
        Md = [0.0,0.0,0.0]
    end
    Fₑ = Fg + Fd
    Mₑ = Mg + Md
    return Fₑ, Mₑ
end