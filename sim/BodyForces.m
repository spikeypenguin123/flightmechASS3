
function [Forces, Moments] = BodyForces(aircraft,X,U,alpha,beta,AngularRates,rho,V)

    % INPUT 
    % aircraft data, 
    % alpha: AoA
    % beta: Sideslip angle
    
    
    % OUTPUT
    % Forces: 
    % Moments: 

    % Computing the Body Forces and moments of the aircraft in the Body axes
    % Content from lecture Week 3A and Week 8B
    % NOTE: Need to check whether thrust input is required

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
    
    % Attitude in Euler angles 
    att_eul = q2e(X(7:10));
    phi = att_eul(1);
    the = att_eul(2);
    psi = att_eul(3);
    
    % Calculating transformation from stability to body
    Cbs = Cx(0)*Cy(alpha)*Cz(0);

    % Controls (DO WE NEED THRUST IN THIS)
    delta_e = U(2); % CHECK (not sure if this needs to the updated one inthe loops it is propbs being used inn of teh reference control elev in the aircraft struct)
    delta_T = U(3);
    delta_a = aircraft.controls.delta_a;
    delta_r = aircraft.controls.delta_r;
    
    S = aircraft.geo.S;
    b = aircraft.geo.b;
    c = aircraft.geo.c;

    Q = 0.5*rho*V^2;

    % Non-dimensionalising
    qhat = (q*c)/(2*V);
    phat = (p*c)/(2*V);
    rhat = (r*c)/(2*V);

    alpha_dot = AngularRates(1);
    beta_dot = AngularRates(2);
    alphadot_hat = (AngularRates(1)*c)/(2*V);
    betadot_hat = (AngularRates(2)*c)/(2*V);

    % Force Coefficients
    CY = CYb*beta + CYbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;  % Removed CYo
    [CL,CD] = WindForces(aircraft,X,U,alpha,V,AngularRates);

    % Forces
    Cx = Cbs*-CD;
    Cy = Cbs*CY;
    Cz = Cbs*-CL;
    
    Fx = Q*S*Cx;
    Fy = Q*S*Cy;
    Fz = Q*S*Cz;

    % Calcualting moment coefficients
    Cl = Clb*beta + Clbd*beta_dot + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;  % Removed Clo
    Cm = Cm0 + Cma*alpha + Cmalpha*alpha_dot + Cmq*qhat + Cmde*delta_e;
    Cn = Cnb*beta + Cnbd*beta_dot + Cnr*rhat + Cnp*phat + Cnda*delta_a + Cndr*delta_r;  % Removed Cno

    % Moments
    L_moment = Cbs*Cl*Q*S*b;
    M = Cbs*Cm*Q*S*c;
    N = Cbs*Cn*Q*S*b;

    Forces = [-D Y L];
    Moments = [L_moment M N]; 

end