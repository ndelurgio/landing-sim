## Gravity
function get_gravity(m,qⁱᵇ, g=-9.8)
    Fgⁱ = [0.0, 0.0, m * g]
    Fg = qⁱᵇ * Fgⁱ ## Body Frame
    Mg = [0.0,0.0,0.0]
    return Fg, Mg
end

function get_enviornment(x)
    Fg, Mg = get_gravity(x.m,QuatRotation(x.qⁱᵇ))
    Fₑ = Fg
    Mₑ = Mg
    return Fₑ, Mₑ
end