function x_next = Integrate(state, t_step)
    % In: state vector, t_step
    % Out: next state vector
    % NOTE: This is just pseudocode until StateRates is implemented.
    
    dx_1 = StateRates(state);
    A = dx_1.*t_step;
    
    dx_2 = StateRates(state+A./2);
    B = dx_2.*t_step;
    
    dx_3 = StateRates(state+B./2);
    C = dx_3.*t_step;
    
    dx_4 = StateRates(state+C);
    D = dx_4.*t_step;

    x_next = state + 1/6.*(A+2*B+2*C+D);
    
end

