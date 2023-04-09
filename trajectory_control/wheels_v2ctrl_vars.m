
function ctrl_vars = wheels_v2ctrl_vars(v_wheels, L)
    syms V w
    eqn = [v_wheels(1) == (V + L/2*w);v_wheels(1) == (V - L/2*w)];
    
    ctrl_vars = solve(eqn, [V,w]);
end