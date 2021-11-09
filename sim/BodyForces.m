
function [Forces, Moments] = BodyForces(aircraft,X,U,alpha,beta,angular_rates,rho,V)
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
    att_eul = q2e(X(7:10)/norm(X(7:10)));
    phi = att_eul(1);
    the = att_eul(2);
    psi = att_eul(3);
    
    % Calculating transformation from stability to body
    Cbs = Cx(0)*Cy(alpha)*Cz(0);
%     Cba = Cx(0)*Cy(alpha)*Cz(-beta);

    % Controls (DO WE NEED THRUST IN THIS)
    delta_e = U(2); % CHECK (not sure if this needs to the updated one inthe loops it is propbs being used inn of teh reference control elev in the aircraft struct)
    delta_T = U(1);
    delta_a = aircraft.controls.delta_a;
    delta_r = aircraft.controls.delta_r;
    
    S = aircraft.geo.S;
    b = aircraft.geo.b;
    c = aircraft.geo.c;

    Q = 0.5*rho*V^2;

    % Non-dimensionalising
    % adam: changed phat and rhat to reference b, not c
    qhat = (q*c)/(2*V);
    phat = (p*b)/(2*V);
    rhat = (r*b)/(2*V);

    alpha_dot = angular_rates(1);
    beta_dot = angular_rates(2);
    alphadot_hat = (angular_rates(1)*c)/(2*V);
    betadot_hat = (angular_rates(2)*c)/(2*V);

    % Force Coefficients
    CY = CYb*beta + CYbd*betadot_hat + CYr*rhat + CYp*phat + CYda*delta_a + CYdr*delta_r;
    [CL,CD] = WindForces(aircraft,X,U,alpha,V,angular_rates);
    
    % Need to check how to implement additional thrust term
%     rho_SL = 1.225;
%     sigma = rho/rho_SL;
%     Pmax_SL = aircraft.prop.P_max; % Data from flight condition
%     Pmax = Pmax_SL*(1.1324*sigma - 0.1324); % Max Power
%     F_T = ((aircraft.prop.eta*Pmax)/V)*delta_T;
%     C_dT = (F_T*c)/(2*V);     
%     + [C_dT ; 0 ; 0]*delta_T
    
    % Forces 
    F_coeff = Cbs*[-CD ; CY ; -CL];
    F_Cx = F_coeff(1);
    F_Cy = F_coeff(2);
    F_Cz = F_coeff(3);
    
    
    Fx = Q*S*F_Cx;
    Fy = Q*S*F_Cy;
    Fz = Q*S*F_Cz;
    
    % Fx *looks* like its off, but thats just because we add thrust later
    % than michael. we all g
    Forces = [Fx Fy Fz];

    
    % Calcualting moment coefficients
    Cl = Clb*beta + Clbd*betadot_hat + Clr*rhat + Clp*phat + Clda*delta_a + Cldr*delta_r;  
    Cm = Cm0 + Cma*alpha + Cmad*alphadot_hat + Cmq*qhat + Cmde*delta_e;
    Cn = Cnb*beta + Cnbd*betadot_hat + Cnr*rhat + Cnp*phat + Cnda*delta_a + Cndr*delta_r;  % Removed Cno
    % Moments
    % TODO: Mx and Mz dont match, but the coefficients are correct. dont know
    % whats going on
    
    Moments = Q*S*[b*Cl; c*Cm; b*Cn];
    


end