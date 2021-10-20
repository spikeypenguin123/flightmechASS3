function [ Rates ] = StateRates([States], [Inertial], [Euler])

    % INPUT:
    % OUTPUT:

    %% Computing the time derivative of each state rate
    % State rates --> x(tn) = ⃗xn = [ un vn wn pn qn rn q0n q1n q2n q3n xen yen zen ]T .

    u = States(1);
    v = States(2);
    w = States(3);
    p = States(4);
    q = States(5);
    r = States(6);
    q0 = States(7);
    q1 = States(8);
    q2 = States(9);
    q3 = States(10);
    q4 = States(11);
    xe = States(12);
    ye = States(13);
    ze = States(14);

    Ixx = Inertial(1);
    Iyy = Inertial(2);
    Izz = Inertial(3);
    Ixz = Inertial(4);

    phi = Euler(1);
    theta = Euler(2);
    psi = Euler(3);


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
    Rates = [udot vdot wdot pdot qdot rdot q0dot q1dot q2dot q3dot xedot yedot zedot]';

end