
function [Forces, Moments] = BodyForces(aircraft,X,U,alpha,beta,angular_rates,V)
    rho = 1.1889;
    % INPUT 
    % aircraft data, 
    % alpha: AoA
    % beta: Sideslip angle
    
    
    % OUTPUT
    % Forces: 
    % Moments: 

    % Computing the Body Forces and moments of the aircraft in the Body axes
    % Content from lecture Week 3A and Week 8B

    % Reading in data
    Cm0 = aircraft.aero.Cmo;
    Cma = aircraft.aero.Cma;
    Cmad = aircraft.aero.Cmad;
    Cmq = aircraft.aero.Cmq;
    Cmde = aircraft.aero.Cmde;
    CYb = aircraft.aero.Cyb;
    CYbd = aircraft.aero.Cybd;
    CYr = aircraft.aero.Cyr;
    CYp = aircraft.aero.Cyp;
    CYda = aircraft.aero.Cyda;
    CYdr = aircraft.aero.Cydr;
    Clb = aircraft.aero.Clb;
    Clbd = aircraft.aero.Clbd;
    Clr = aircraft.aero.Clr;
    Clp = aircraft.aero.Clp;
    Clda = aircraft.aero.Clda;
    Cldr = aircraft.aero.Cldr;
    Cnb = aircraft.aero.Cnb;
    Cnbd = aircraft.aero.Cnbd;
    Cnr = aircraft.aero.Cnr;
    Cnp = aircraft.aero.Cnp;
    Cnda = aircraft.aero.Cnda;
    Cndr = aircraft.aero.Cndr;

    p = X(4);
    q = X(5);
    r = X(6);
    
    % Calculating transformation from stability to body
    Cbs = Cx(0)*Cy(alpha)*Cz(0);
    
    % Controls (note we don't need delta_T as we already know thrust force)
    delta_e = U(2);
    delta_a = aircraft.controls.delta_a;
    delta_r = aircraft.controls.delta_r;
    
    S = aircraft.geo.S;
    b = aircraft.geo.b;
    c = aircraft.geo.c;

    Q = 0.5*rho*V^2;

    % Non-dimensionalising
    qhat = (q*c)/(2*V);
    phat = (p*b)/(2*V);
    rhat = (r*b)/(2*V);

    alphadot_hat = (angular_rates(1)*c)/(2*V);
    betadot_hat = (angular_rates(2)*c)/(2*V);

    % Force Coefficients
    CY = CYb*beta + CYbd*betadot_hat + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
    [CL,CD] = WindForces(aircraft,X,U,alpha,V,angular_rates);
    

    % Forces 
    F_coeff = Cbs*[-CD ; CY ; -CL];
    F_Cx = F_coeff(1);
    F_Cy = F_coeff(2);
    F_Cz = F_coeff(3);
    
    
    Fx = Q*S*F_Cx;
    Fy = Q*S*F_Cy;
    Fz = Q*S*F_Cz;
    
    Forces = [Fx Fy Fz];

    
    % Calcualting moment coefficients
    Cl = Clb*beta + Clbd*betadot_hat + Clr*rhat + Clp*phat + Clda*delta_a + Cldr*delta_r;  
    Cm = Cm0 + Cma*alpha + Cmad*alphadot_hat + Cmq*qhat + Cmde*delta_e;
    Cn = Cnb*beta + Cnbd*betadot_hat + Cnr*rhat + Cnp*phat + Cnda*delta_a + Cndr*delta_r;
    
    % Moments
    Moments = Q*S*[b*Cl; c*Cm; b*Cn];
end