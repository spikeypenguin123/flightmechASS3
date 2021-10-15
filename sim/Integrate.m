function x_next = Integrate(state, t_step)
    % In: state vector, t_step
    % Out: next state vector
    % NOTE: This is just pseudocode until StateRates is implemented.
    
    dx_1 = StateRates(state);
    A_n = dx_1.*t_step;
    
    dx_2 = StateRates(state+A_n./2);
    B_n = dx_2.*t_step;
    
    dx_3 = StateRates(state+B_n./2);
    C_n = dx_3.*t_step;
    
    dx_4 = StateRates(state+C_n);
    D_n = dx_4.*t_step;

    x_next = state + 1/6.*(A_n+2*B_n+2*C_n+D_n);
    
end

