## Initial State
p₀ = [0.0, 0.0, 0.0]
v₀ = [0.0, 0.0, 0.0]
ω₀ = [0.0, 0.0, 0.0]
q₀ = [1.0, 0.0, 0.0, 0.0]

## Initial Vehicle Properties
m₀ = 200.0
I₀ = [
    100.0    10.0     0.0
    10.0     100.0    0.0
    0.0      0.0      200.0
    ]

## Simulation Timespan
tspan = (0,50)

## Init State
vehicle = ComponentArray(
            pⁱ  = p₀, 
            vⁱ  = v₀, 
            ωᵇ  = ω₀, 
            qⁱᵇ = q₀,
            m   = m₀,
            Iᵇ  = I₀)