% Function to do ALL the Forces


function [CL, Grav, Thrust, Body] = AllForces(aircraft,X,U,)
    
    [alpha, beta, V] = AeroAngles(X);

    [rho, Q] = FlowProperties(altitude, V, g);
    
    % Calcuate Body Forces
    [Forces, Moments] = BodyForces(FlightData,alpha,beta,alphadot,betadot,rho,V,[States],[Controls]);
    
    % Calcuate Gravity Forces
    G_body = Gravity(g,q,aircraft.inertial.m);
end