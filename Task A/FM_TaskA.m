clc
clear
close all

%% LOADING DATA

[ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();
% [ FlightData ] = aero3560_LoadFlightDataPC9_CG2();

Inert = FlightData.Inertial;
CLim = FlightData.ControlLimits;
Aero = FlightData.Aero;
Geom = FlightData.Geo;

%% COMPUTING BASIC VARIABLES

% alpha = -10:0.5:10;
% Cl = (alpha - Aero.alpha_o)*Aero.CLa;
xac = 0.25;
g = Inert.g;
m = Inert.m;
W = m*g;
V = 100/1.944; % Speed at 100 knots in m/s
rho_US = 0.9711*0.002377; % Density at 1000ft
rho = rho_US*515;
Q = 0.5*rho*(V^2)*Geom.S;

%% TASK 1

% Initialising headings
psi_a = deg2rad(-25);
gamma = deg2rad(-7);
theta_a = gamma;
beta = deg2rad(-7);

% Computing stability equation in order to find equlibrium CL

% Setting convergence parameters
kmax = 500;
eps = 10^-3;

% Initial guesses
Cl0 = 0.5;

Cl = W/Q;
for k = 1:kmax
    while error > eps
        
        
        
        
    end
end
