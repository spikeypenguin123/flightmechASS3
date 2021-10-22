function Xd = StateRates(X, U, aircraft, AngularRates)

    % INPUT:
    % X: State vector
    % Xd0: State vector (All zero when it is called in Trimrates.m initially)
    % U: Control vector
    % aircraft data
    
    % OUTPUT:
    % Computing the time derivative of each state rate
    % State rates --> x(tn) = ⃗xn = [ un vn wn pn qn rn q0n q1n q2n q3n xen yen zen ]T .

    % State vector
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
    xe = X(11);
    ye = X(12);
    ze = X(13);
    
    % Attitude in Euler angles 
    att_eul = q2e(X(7:10));
    phi = att_eul(1);
    the = att_eul(2);
    psi = att_eul(3);
    
    % Inertial Data
    Ixx = aircraft.inertial.Ixx;
    Iyy = aircraft.inertial.Iyy;
    Izz = aircraft.inertial.Izz;
    Ixz = aircraft.inertial.Ixz;
    m = aircraft.inertial.m;
    
    % Forces 
    [CL, CD, F_B, M_B, F_G, F_T, Pmax] = AllForces(aircraft,X,U,AngularRates);
    

    % Velocity time derivatives
    % m = 
    % F_A = 
    % F_T = 

    % udot = (m*gx + F_Ax + F_Tx)/m + r*v - q*w;
    % vdot = (m*gy + F_Ay + F_Ty)/m - r*u + p*w;
    % wdot = (m*gz + F_Az + F_Tz)/m + q*u - p*v;

    udot = r*v - q*w - g*sin(theta) + F_x/m;
    vdot = -r*u + p*w + g*sin(phi)*cos(theta) + F_y/m;
    wdot = q*u - p*v + g*cos(phi)*cos(theta) + F_z/m;


    % Body rates time derivatives
    C0 = Ixx/Izz - Ixz^2;
    C1 = Izz/C0;
    C2 = Ixz/C0;
    C3 = C2*(Ixx - Iyy + Izz);
    C4 = C1*(Iyy - Izz) - C2*Ixz;
    C5 = 1/Iyy;
    C6 = C5*Ixz;
    C7 = C5*(Izz - Ixx);
    C8 = Ixx/C0;
    C9 = C8*(Ixx - Iyy) + C2*Ixz;

    %%%%%% Need to define moments
    % L = M_Ax + M_Tx;
    % M = M_Ay + M_Ty;
    % N = M_Az + M_Tz;

    pdot = C3*p*q + C4*q*r + C1*L + C2*N;
    qdot = C7*p*r - C6*(p^2 - r^2) + C5*M;
    rdot = C9*p*q - C3*q*r + C2*L + C8*N;


    % Quaternion time derivatives
    q0dot = -0.5*(q1*p + q2*q + q3*r);
    q1dot = 0.5*(q0*p - q3*q + q2*r);
    q2dot = 0.5*(q3*p + q0*q - q1*r);
    q3dot = -0.5*(q2*p - q1*q - q0*r);


    % Position time derivatives
    positiondot = (Cz(-psi)*Cy(-theta)*Cx(-phi))*[u v w]';
    xedot = positiondot(1);
    yedot = positiondot(2);
    zedot = positiondot(3);

    % xedot = (cos(psi)*cos(theta))*u + (cos(psi)*sin(phi) - sin(psi)*cos(phi))*v + (cos(psi)*sin(theta)*cos(phi) + sin(psi)*sin(phi))*w;
    % yedot = (sin(psi)*cos(theta))*u + (sin(psi)*sin(theta)*sin(phi) + cos(pi)*cos(phi))*v + (sin(psi)*sin(theta)*cos(phi) - cos(psi)*sin(phi))*w;
    % zedot = -sin(theta)*u + (cos(theta)*sin(phi))*v + (cos(theta)*cos(phi))*w;


    % Time derivative state vector
    Xd = [udot vdot wdot pdot qdot rdot q0dot q1dot q2dot q3dot xedot yedot zedot]';

end