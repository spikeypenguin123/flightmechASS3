function [CL,CD] = WindForces(aircraft,X,U,alpha,V,AngularRates)

    % INPUT:
    % OUTPUT:
    % CL: lift coeff
    % CD: drag coeff
    
    % Computing Wind Forces
    % Using material from lecture Week 8B

    % Coefficients to compute lift coefficient
    CL0 = aircraft.aero.CLo;
    CLa = aircraft.aero.CLa;
    CLq = aircraft.aero.CLq;
    CLde = aircraft.aero.CLde;
    CLad = aircraft.aero.CLad;

    % Coefficients to compute drag coefficients
    Cdo = aircraft.aero.Cdo;
    k = aircraft.aero.k;

    c = aircraft.geo.c;

    % Angular Position/Rates and Controls
    alphadot = AngularRates(1);
    q = X(5);
    delta_e = U(2);

    % Normalising States and Controls
    alphadot_hat = (alphadot*c)/(2*V);
    q_hat = (q*c)/(2*V);

    % Computing lift and drag coefficients
    CL = CL0 + CLa*alpha + CLad*alphadot_hat + CLq*q_hat + CLde*delta_e;
    CD = Cdo + k*CL^2;

end