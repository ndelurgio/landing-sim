function get_guidanceCmd(x,t)
    if x.pⁱ[3] < 25
        v_cmd = -0.5
    else
        v_cmd = -10
    end

    return v_cmd
end