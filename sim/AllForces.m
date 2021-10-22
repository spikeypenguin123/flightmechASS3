% Function to do ALL the Forces


function [CL, CD, Grav, Thrust, Body, rho, Q] = AllForces(aircraft,X,U,AngularRates)
    
    % AoA, sideslip and velocity
    [alpha, beta, V] = AeroAngles(X);

    [rho, Q] = FlowProperties(aircraft,V);
    
    % Wind Forces
    [CL,CD] = WindForces(aircraft,alpha,V,AngularRates,X,U);
    
    % Body Forces
    [Forces, Moments] = BodyForces(aircraft,alpha,beta,AngularRates,rho,V,X,U);    
    
    % Gravity Forces
    G_body = Gravity(aircraft.inertial.g,[X(7);X(8);X(9);X(10)],aircraft.inertial.m);
    
    % Thrust Forces
    [Pmax, T] = PropForces(rho, V, aircraft.controls.delta_T, aircraft.prop);

end