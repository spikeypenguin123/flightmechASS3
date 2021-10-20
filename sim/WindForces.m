function [CL,CD] = WindForces(Aero,alpha,V,[AngularRates],[States],[Controls])

    % INPUT:
    % OUTPUT:

    %% Computing Wind Forces
    % Using material from lecture Week 8B

    % Coefficients to compute lift coefficient
    CL0 = Aero.CL0;
    CLa = Aero.CLa;
    CLq = Aero.CLq;
    CLde = Aero.CLde;
    CLad = Aero.CLad;

    % Coefficients to compute drag coefficients
    Cdo = Aero.Cdo;
    k = Aero.k;

    c = FlightData.Geom.c;

    % Angular Position/Rates and Controls
    alphadot = AngularRates(1);
    q = States(5);
    delta_e = Controls(2);

    % Normalising States and Controls
    alphadot_hat = (alphadot*c)/(2*V);
    q_hat = (q*c)/(2*V);

    % COmputing lift and drag coefficients
    CL = CL0 + CLa*alpha + CLad*alphadot + CLq*q_hat + Clde*delta_e;
    CD = Cdo + k*CL^2;

end