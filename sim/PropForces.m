function  [Pmax, Thrust] = PropForces(sigma, eta, V_T, delta_T, Pmax_SL)
    
    % Computing Propulsion Fores
    
    Pmax = Pmax_SL*(1.1324*sigma - 0.1324); % Max Power
    Thrust = ((eta*Pmax)/V_T)*delta_T; % Thrust


end