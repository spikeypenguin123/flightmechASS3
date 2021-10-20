clc
clear
close all

%% LOADING DATA

[ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();
% [ FlightData ] = aero3560_LoadFlightDataPC9_CG2();

Inertial = FlightData.Inertial;
CLim = FlightData.ControlLimits;
Aero = FlightData.Aero;
Geom = FlightData.Geo;

%% COMPUTING BASIC VARIABLES

CLo = Aero.CLo;
CLa = Aero.CLa;
g = Inertial.g;
m = Inertial.m;


W = m*g;
V = 100/1.944; % Speed at 100 knots in m/s

% Recalculate density
rho_US = 0.9711*0.002377; % Density at 1000ft
rho = rho_US*515;
Q = 0.5*rho*(V^2);

%% TASK 1 - Part (a): Longitudinal Equilibrium Conditions

% Initialising headings
psi_a = deg2rad(-25);
gamma = deg2rad(-7);
theta_a = gamma;
beta = deg2rad(-7);

% Computing stability equation in order to find equlibrium CL, alpha,
% elevator position and throtte position

% Initial guesses
gamma = 0.5; 
Cl = 0.5;

% Setting convergence parameters
eps = 1e-5;
error = 1;
while error > eps
    
    L = Cl*Q*Geom.S;
    gamma = acos(L/W);
    Cl_next = (W*cos(gamma))/(Q*Geom.S);
    
    error = abs(Cl_next - Cl);
    Cl = Cl_next;
    
end

Cleq = W/(Q*Geom.S);


a0 = (Cl - CLo)/CLa;
delta_T0 = 0.5;
delta_e0 = 0;

% Setting convergence parameters
error = 1;
while error > eps
    
    
    
    
end

%% Task 1 - Part (b): Lateral-directional equilibrium conditions



%% Task 1 - Part (c): Euler Angles
