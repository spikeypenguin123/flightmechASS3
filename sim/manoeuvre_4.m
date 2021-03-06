% manoeuvre_4 loop
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
rhoSSL = 1.225;

% Man 4
nz = 3.5;
L = nz/(m*g);
CL = (2*L)/(rho*(V^2)*S);

dCmcgdCL = 0.6465;

Cmq = Aero.Cmq;     % Pitch damping coefficient
cbar = Geom.c;      % Mean Aerodynamic Chord

% is Cmde same as Cmde_bar?
de = -((m*g/S)/(0.5*rhoSSL*V^2*Cmde) * (dCmcgdCL*nz + Cmq*(cbar*rho/(4*m/S))*(nz-1)));


% Print for Question 1
disp('-------------------');
fprintf('Estimated controls for Case 4 %2.1fg loop\n',nz);
fprintf('de = %2.3f (deg) \n', rad2deg(de));


% Control_GUI LoopNomCG

% save test.mat
% save('LoopNomCGV2.mat','U_filter')