clc
clear
close all

%% LOADING DATA

[ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();

% Renaming structs
Inertial = FlightData.Inertial;
CLim = FlightData.ControlLimits;
Aero = FlightData.Aero;
Geom = FlightData.Geo;
Prop = FlightData.Prop;

%% COMPUTING BASIC VARIABLES

% Loading in data
CLo = Aero.CLo;
CLa = Aero.CLa;
CLde = Aero.CLde;
Cmo = Aero.Cmo;
Cma = Aero.Cma;
Cmde = Aero.Cmde;
Cyda = Aero.Cyda;
Cydr = Aero.Cydr;
Clda = Aero.Clda;
Cldr = Aero.Cldr;
Cnda = Aero.Cnda;
Cndr = Aero.Cndr;
Cyb = Aero.Cyb;
Clb = Aero.Clb;
Cnb = Aero.Cnb;
Cdo = Aero.Cdo;
k = Aero.k;

g = Inertial.g;
m = Inertial.m;
eta = Prop.eta;
Pmax_SL = Prop.P_max;

% Flight characteristics
W = m*g;
V = 100/1.944; % Speed at 100 knots in m/s
% Recalculate density
rho_US = 0.9711*0.002377; % Density at 1000ft
rho = rho_US*515;
Q = 0.5*rho*(V^2);

% Initialising headings
psi_a = deg2rad(-25);
gamma = deg2rad(-7);
theta_a = gamma;
beta = deg2rad(-7);

%% TASK 1 - Part (a): Longitudinal Equilibrium Conditions

% Computing stability equation in order to find equlibrium CL, alpha,
% elevator position and throtte position

% Equilbrium lift coefficient when Lift = Weight
CL = W/(Q*Geom.S);

% Running an iteration to get the the equilibrium CL
eps = 1e-5;
error = 1;
while error > eps
    
    L = CL*Q*Geom.S;
    gamma = acos(L/W);
    Cl_next = (W*cos(gamma))/(Q*Geom.S);
    
    error = abs(Cl_next - CL);
    CL = Cl_next;
    
end

% Computing the zero-lift angle of attack
a0 = (CL - CLo)/CLa;
alpha_check = rad2deg(a0 + CL/CLa);

% Calculating simultaneous equation using material from week 8B
A_pa = [CLa CLde ; 
        Cma Cmde];
b_pa = [CL - CLo ; - Cmo];

% Solving the system of equations for the angle of attack and elevator
% deflection
x_pa = A_pa\b_pa;
alpha = x_pa(1);
de = x_pa(2);

% Using propulsion model from assignment sheet
Cd = Cdo + k*CL^2;
D = Cd*Q*Geom.S;
T = D; % Equilibrium condition for level flight
Pmax = Pmax_SL*(1.1324*(rho/1.225) - 0.1324); % Computing maximum power
dT = (T*V)/(eta*Pmax); % Throttle position


disp('------------------');
disp('Part a: Longitudinal directional equilibrium conditions');
disp(['Equilibrium Lift Coefficient: ', num2str(CL)]);
disp(['Angle of attack: ', num2str(rad2deg(alpha)), ' deg']);
disp(['Elevator Deflection: ', num2str(rad2deg(de)), ' deg']);
disp(['Throttle Position: ', num2str(rad2deg(dT)), ' deg']);


%% Task 1 - Part (b): Lateral-directional equilibrium conditions

% Solving simultaneous equations from lecture 7B to compute bank angle,
% aileron deflection and rudder deflection
% Note that CYo = Clo = Cno = 0 in a symmetric aircraft

A_pb = [Cyda Cydr CL;
     Clda Cldr 0;
     Cnda Cndr 0];
b_pb = -beta.*[Cyb ; Clb ; Cnb];

% Solving system of equation to compute aileron deflection, rudder
% deflection and bank angle
x_pb = A_pb\b_pb;
da = x_pb(1);
dr = x_pb(2);
phi = x_pb(3);


disp('-------------------');
disp('Part b: Lateral directional equilibrium conditions');
disp(['Aileron deflection: ', num2str(rad2deg(da)), ' deg']);
disp(['Rudder deflection: ', num2str(rad2deg(dr)), ' deg']);
disp(['Bank angle: ', num2str(rad2deg(phi)), ' deg']);


%% Task 1 - Part (c): Euler Angles

% Convert body angle to Euler Angles
% Transformation matrix from airpath to body is simply Cy(a)Cz(-b)
% LVLH to to body Cx(phi)Cy(a)Cz(psi)
% Air path to LVLH Cx(phia)Cy(theta)Cz(psia)

Cba = Cy(alpha)*Cz(-beta);
Cva = Cx(phi)*Cy(theta_a)*Cz(psi_a);

% Using aircraft diagram from week 1B Slide 25
% Determining Euler Angles of body axes with respect to LVLH
phi_v = -phi;
theta_v = theta_a + alpha;
psi_v = psi_a + -beta;


disp('-------------------');
disp('Part c: Euler Angles of body axes with respect to LVLH');
disp(['Roll angle (phi): ', num2str(rad2deg(phi_v)), ' deg']);
disp(['Pitch angle (theta): ', num2str(rad2deg(theta_v)), ' deg']);
disp(['Yaw angle (psi): ', num2str(rad2deg(psi_v)), ' deg']);

