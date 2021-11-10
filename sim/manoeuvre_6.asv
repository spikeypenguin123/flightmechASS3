% manoeuvre_6 
close all 
clear all 
clc

% Load data
addpath('AircraftData');
load('Longitudinal_Matrices_PC9_nominalCG1_180Kn_1000ft.mat');
FD = aero3560_LoadFlightDataPC9_nominalCG1();


% 5 deg steady heading sideslip
beta = deg2rad(5);

% Straight, Steady, Constant Altitude for 15 seconds

% FD.Aero.a = FD.Aero.aw + FD.Aero.e*FD.Geom.St/FD.Geom.S*FD.Aero.at;

% FD.Aero.CL = FD.Aero.CLa*FD.Aero.a + FD.Aero.CLadot*FD.Aero.adot; + FD.Aero.CLq*FD.Aero.q + FD.Aero.CLde * FD.Aero.de;

L = FD.Inertial.m*FD.Inertial.g;
CL = (2*L)/(FD.Oper.rho*(FD.Oper.V^2)*FD.Geom.S);


%% Q1
% Bank angle, Aileron Deflection, Rudder Deflection to maintain a
% steady-heading sideslip
A = [CL,0,FD.Aero.CYdr;
    0,FD.Aero.Clda,FD.Aero.Cldr;
    0,FD.Aero.Cnda,FD.Aero.Cndr];

y = -[FD.Aero.CYb;FD.Aero.Clb;FD.Aero.Cnb].*beta;

% Solve for the bank angle, da, dr
x = linsolve(A,y);

% Then the aileron deflection 
phi= x(1);
da = x(2);
dr = x(3);

% Print for Question 1
fprintf('Question 1 for sideslip of %2.2f deg:\n',rad2deg(beta));
fprintf('phi = %2.3f (deg) \n', rad2deg(phi));
fprintf('delta_a = %2.3f (deg) \n', rad2deg(da));
fprintf('delta_r = %2.3f (deg) \n\n', rad2deg(dr));


