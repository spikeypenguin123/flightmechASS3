
function [Forces, Moments] = BodyForces(aircraft,alpha,beta,AngularRates,rho,V,X,U)

    % INPUT 
    % aircraft data, 
    % alpha: AoA
    % beta: Sideslip angle
    
    
    % OUTPUT
    % Forces: 
    % Moments: 

    % Computing the Body Forces and moments of the aircraft in the Body axes
    % Content from lecture Week 3A and Week 8B
    % NOTE: Still need to do rotation on matrix 20/10

    % Reading in data
    Cm0 = aircraft.Aero.Cmo;
    Cma = aircraft.Aero.Cma;
    Cmad = aircraft.Aero.Cmad;
    Cmq = aircraft.Aero.Cmq;
    Cmde = aircraft.Aero.Cmde;
    % CY0 = aircraft.Aero.;
    CYb = aircraft.Aero.Cyb;
    CYbd = aircraft.Aero.Cybd;
    CYr = aircraft.Aero.Cyr;
    CYp = aircraft.Aero.Cyp;
    CYda = aircraft.Aero.Cyda;
    CYdr = aircraft.Aero.Cydr;
    % Cl0 = aircraft.Aero.;
    Clb = aircraft.Aero.Clb;
    Clbd = aircraft.Aero.Clbd;
    Clr = aircraft.Aero.Clr;
    Clp = aircraft.Aero.Clp;
    Clda = aircraft.Aero.Clda;
    Cldr = aircraft.Aero.Cldr;
    % Cn0 = aircraft.Aero.;
    Cnb = aircraft.Aero.Cnb;
    Cnbd = aircraft.Aero.Cnbd;
    Cnr = aircraft.Aero.Cnr;
    Cnp = aircraft.Aero.Cnp;
    Cnda = aircraft.Aero.Cnda;
    Cndr = aircraft.Aero.Cndr;

    p = X(4);
    q = X(5);
    r = X(6);

    delta_e = U(2);
    delta_a = U(3);

    S = aircraft.Geom.S;
    b = aircraft.Geom.b;
    c = aircraft.Geom.c;

    Q = 0.5*rho*V^2;

    % Non-dimensionalising
    qhat = (q*c)/(2*V);
    phat = (p*c)/(2*V);
    rhat = (r*c)/(2*V);

    alphadot_hat = (AngularRates(1)*c)/(2*V);
    betadot_hat = (AngularRates(2)*c)/(2*V);

    % Force Coefficients
    CY = CY0 + CYb*beta + CYbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
    [CL,CD] = WindForces(aircraft,alpha,V,AngularRates,X,U);

    % Forces
    D = CD*Q*S;
    Y = CY*Q*S;
    L = CL*Q*S;

    % Calcualting moment coefficients
    Cl = Cl0 + Clb*beta + Clbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
    Cm = Cm0 + Cma*alpha + Cmalpha*alpha_dot + Cmq*qhat + Cmde*delta_e;
    Cn = Cn0 + Cnb*beta + Cnbd*beta_dot + Cnr*rhat + Cnp*phat + Cnda*delta_a + Cndr*delta_r;

    % Moments
    L_moment = Cl*Q*S*b;
    M = Cm*Q*S*c;
    N = Cn*Q*S*b;

    Forces = [-D Y L];
    Moments = [L_moment M N]; 

end