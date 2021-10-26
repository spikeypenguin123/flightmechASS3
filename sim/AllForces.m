% Function to do ALL the Forces


function [CL, CD, F_B, M_B, F_G, F_T, Pmax] = AllForces(aircraft,X,U,angular_rates)
    
    % AoA, sideslip and velocity
    [alpha, beta, V] = AeroAngles(X);

    [rho, ~] = FlowProperties(aircraft,V);
    
    % Wind Forces
    [CL,CD] = WindForces(aircraft,X,U,alpha,V,angular_rates);
    
    % Body Forces
    [F_B, M_B] = BodyForces(aircraft,X,U,alpha,beta,angular_rates,rho,V);
    
    % Gravity Forces
    F_G = Gravity(aircraft.inertial.g,[X(7);X(8);X(9);X(10)],aircraft.inertial.m);
    
    % Thrust Forces
    [Pmax, F_T] = PropForces(rho, V, aircraft.controls.delta_T, aircraft.prop);

end