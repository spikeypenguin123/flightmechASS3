function [rho, Q] = FlowProperties(aircraft,V)
    % Input: aircraft data, velocity (m/s)
    % Output: 
    % rho: density (kg/m^2) 
    % Q: dynamic pressure (N/m^2)
    % Note: only considers troposphere (altitude<=11km)
    
    g = aircraft.inertial.g;
    X = PullState(aircraft);        % Get the state vector 
    altitude = -X(13);              % neg z_e
    
    % Constants (from literature):
    P_0 = 101325;         % Pressure at sea level       (Pa)
    T_0 = 15+273.15;      % Temperature at sea level    (K)
    T_11 = -56.5+273.15;  % Temperature at 11km         (K)
    R = 287;              % Gas constant,               (m^2/s^2/K)
    
    % assuming linear distribution of temp we find the lapse rate:
    L = (T_0-T_11)/11000;
    
    % we now find the air properties at arbitrary altitude
    T = T_0-L*altitude;
    P = P_0*(T/T_0)^(g/L/R);
    rho = P/R/T;
    Q = 0.5*rho*V^2;
end

