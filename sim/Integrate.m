function [x_next, dx] = Integrate(aircraft, t_step, dx_prev, debug)
    % In: state vector, t_step
    % Out: next state vector
    % NOTE: This is just pseudocode until StateRates is implemented.
    
    [state, U, ~] = get_vectors(aircraft);
    
    aa = AngularRates(state, dx_prev);
    dx_1 = StateRates(aircraft, state, U, aa);
    A = dx_1.*t_step;
    
    if debug
        % only do the one passthru
        x_next = state + A;
        dx = A;
        return
    end
    
    aa = AngularRates(state, dx_1);
    dx_2 = StateRates(aircraft, state+A./2, U, aa);
    B = dx_2.*t_step;
    
    aa = AngularRates(state, dx_2);
    dx_3 = StateRates(aircraft, state+B./2, U, aa);
    C = dx_3.*t_step;
    
    aa = AngularRates(state, dx_3);
    dx_4 = StateRates(aircraft, state+C, U, aa);
    D = dx_4.*t_step;

    dx = 1/6.*(A+2*B+2*C+D);
    x_next = state + dx;
    
end

