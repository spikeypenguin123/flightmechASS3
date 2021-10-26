function Xd = StateRates(aircraft, X, U, angular_rates)

    % INPUT:
    % X: State vector
    % Xd0: State vector (All zero when it is called in Trimrates.m initially)
    % U: Control vector
    % aircraft data
    
    % OUTPUT:
    % Computing the time derivative of each state rate
    % State rates --> x(tn) = ⃗xn = [ un vn wn pn qn rn q0n q1n q2n q3n xen yen zen ]T .
    
    % Attitude in Euler angles 
    att_eul = q2e(X(7:10));
    phi = att_eul(1);
    theta = att_eul(2);
    psi = att_eul(3);
    
    % Inertial Data
    Ixx = aircraft.inertial.Ixx;
    Iyy = aircraft.inertial.Iyy;
    Izz = aircraft.inertial.Izz;
    Ixz = aircraft.inertial.Ixz;
    m = aircraft.inertial.m;
    g = aircraft.inertial.g;
    
    % State
    u = X(1);
    v = X(2);
    w = X(3);
    p = X(4);
    q = X(5);
    r = X(6);
    q0 = X(7);
    q1 = X(8);
    q2 = X(9);
    q3 = X(10);
    x_e = X(11);
    y_e = X(12);
    z_e = X(13);
    
    % Forces 
    [CL, CD, F_B, M_B, F_G, F_T, Pmax] = AllForces(aircraft,X,U,angular_rates);
    

    % Velocity time derivatives
    F_x = F_B(1);
    F_y = F_B(2);
    F_z = F_B(3);
    
    % double check that the gravity forces correspond to the right
    % array elements
    F_gx = F_G(1); 
    F_gy = F_G(2);
    F_gz = F_G(3);

%     udot = r*v - q*w - g*sin(theta) + (F_x + F_gx + F_T)/m;
%     vdot = -r*u + p*w + g*sin(phi)*cos(theta) + (F_y + F_gy)/m;
%     wdot = q*u - p*v + g*cos(phi)*cos(theta) + (F_z + F_gz)/m;
    
    udot = r*v - q*w + (F_x + F_gx + F_T)/m;
    vdot = -r*u + p*w + (F_y + F_gy)/m;
    wdot = q*u - p*v + (F_z + F_gz)/m;


    % Body rates time derivatives
    C0 = Ixx*Izz - Ixz^2;
    C1 = Izz/C0;
    C2 = Ixz/C0;
    C3 = C2*(Ixx - Iyy + Izz);
    C4 = C1*(Iyy - Izz) - C2*Ixz;
    C5 = 1/Iyy;
    C6 = C5*Ixz;
    C7 = C5*(Izz - Ixx);
    C8 = Ixx/C0;
    C9 = C8*(Ixx - Iyy) + C2*Ixz;

    % Moments
    L = M_B(1);
    M = M_B(2);
    N = M_B(3);

    pdot = C3*p*q + C4*q*r + C1*L + C2*N;
    qdot = C7*p*r - C6*(p^2 - r^2) + C5*M;
    rdot = C9*p*q - C3*q*r + C2*L + C8*N;


    % Quaternion time derivatives
    q0dot = -0.5*(q1*p + q2*q + q3*r);
    q1dot = 0.5*(q0*p - q3*q + q2*r);
    q2dot = 0.5*(q3*p + q0*q - q1*r);
    q3dot = -0.5*(q2*p - q1*q - q0*r);


    % Position time derivatives
    positiondot = (Cz(-psi)*Cy(-theta)*Cx(-phi))*[u v w]'; % Going from body to earth
    xedot = positiondot(1);
    yedot = positiondot(2);
    zedot = positiondot(3);

    % Time derivative state vector
    Xd = [udot vdot wdot pdot qdot rdot q0dot q1dot q2dot q3dot xedot yedot zedot]';

end