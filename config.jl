## Initial State
p₀ = [0.0, 0.0, 0.0]
v₀ = [0.0, 0.0, 0.0]
ω₀ = [0.01, 0.03, 0.0]
q₀ = [1.0, 0.0, 0.0, 0.0]

## Initial Vehicle Properties
m₀ = 22.3
I₀ = [
    4.59     0.0      0.0
    0.0      4.59     0.0
    0.0      0.0      5.3
    ]
cg₀ = [0.0,0.0,0.35]

## Simulation Timespan
tspan = (0,50)

## Main Engine Setup
rotation_misalign = 0.000
offset_misalign = 0.0000
mainEngine = (
    p = [0.0,0.0,0.0],
    axis= RotYZ(offset_misalign, rotation_misalign) * [0.0,0.0,1.0],
    ṁ_max = 0.2,
    isp = 223.163,
    min_throttle=0.2,
    Pₑ=0.0
)
throttle₀ = 0.0
uᵣ₀ = 0.0 ## roll off zero, rad
uₚ₀ = 0.0 ## pitch off zero, rad

## Controller Setup
include("scripts/lqr_design.jl")

## Enable/Disable Disturbances
enable_disturbances = false
bypass_actuators = true

## Init State
vehicle = ComponentArray(
            pⁱ  = p₀, 
            vⁱ  = v₀, 
            ωᵇ  = ω₀, 
            qⁱᵇ = q₀,
            m   = m₀,
            Iᵇ  = I₀,
            cg  = cg₀,
            mainEngine = (
                throttle = throttle₀,
                uᵣ = uᵣ₀,
                uₚ = uₚ₀
            ))

## Init Params
parm =  (
        mainEngine=mainEngine,
        enable_disturbances=enable_disturbances,
        bypass_actuators=bypass_actuators,
        controller=L
        )