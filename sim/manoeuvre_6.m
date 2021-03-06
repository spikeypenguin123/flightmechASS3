% manoeuvre_6 
close all 
clear all 
clc

% Load data
addpath('AircraftData');

[ FlightData ] = aero3560_LoadFlightDataPC9_nominalCG1();
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
S = Geom.S; % wing area

% Flight characteristics
W = m*g;
V = 180/1.944; % Speed at 100 knots in m/s
% Recalculate density
rho_US = 0.9711*0.002377; % Density at 1000ft
rho = rho_US*515;
Q = 0.5*rho*(V^2);



% Initialising headings
psi_a = deg2rad(-25);   % airpath axis heading angle
gamma = deg2rad(0);     % Flight path angle --> case 6 is constant altitude
theta_a = gamma;
beta = deg2rad(5);      % 5 deg steady heading sideslip

% Straight, Steady, Constant Altitude for 15 seconds

% FD.Aero.a = FD.Aero.aw + FD.Aero.e*FD.Geom.St/FD.Geom.S*FD.Aero.at;

% FD.Aero.CL = FD.Aero.CLa*FD.Aero.a + FD.Aero.CLadot*FD.Aero.adot; + FD.Aero.CLq*FD.Aero.q + FD.Aero.CLde * FD.Aero.de;

L = m*g;
CL = (2*L)/(rho*(V^2)*S);


%% Q1
% Bank angle, Aileron Deflection, Rudder Deflection to maintain a
% steady-heading sideslip
% A = [CL,0,FD.Aero.CYdr;
%     0,FD.Aero.Clda,FD.Aero.Cldr;
%     0,FD.Aero.Cnda,FD.Aero.Cndr];
% 
% y = -[FD.Aero.CYb;FD.Aero.Clb;FD.Aero.Cnb].*beta;
A = [Cyda Cydr CL;
     Clda Cldr 0;
     Cnda Cndr 0];
b = -beta.*[Cyb ; Clb ; Cnb];

% Solve for the bank angle, da, dr
x = A\b;       % x = linsolve(A,b);

% Then the aileron deflection 
da = x(1);
dr = x(2);
phi = x(3);

% Print for Question 1
disp('-------------------');
fprintf('Equilibrium state controls for Case 6 sideslip of %2.2f deg:\n',rad2deg(beta));
fprintf('phi = %2.3f (deg) \n', rad2deg(phi));
fprintf('delta_a = %2.3f (deg) \n', rad2deg(da));
fprintf('delta_r = %2.3f (deg) \n\n', rad2deg(dr));


% Control_GUI

% save test.mat
% save('StepOnBall.mat','U_filter')
