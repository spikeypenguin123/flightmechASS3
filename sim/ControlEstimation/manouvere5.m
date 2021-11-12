clc
clear

% Manouvere 5 - Using Week 7B for balanced turn
% Steady 2g zero-sideslip turn, from steady flight

addpath('AircraftData');
addpath('Control_GUI');
FD = aero3560_LoadFlightDataPC9_nominalCG1();
V = 180*0.51444;
g = 9.81;
b = FD.Geo.b;

% 2g bank angle
n = 4;
theta = acos(1/n);

% rate of turn
r = (g*tan(theta))/V;
rhat = (r*b)/(2*V);

% Computing control deflections
A = [FD.Aero.Clda FD.Aero.Cldr ;
     FD.Aero.Cnda FD.Aero.Cndr];
B = -rhat.*[ FD.Aero.Clr ; FD.Aero.Cnr ];

x = A\B;

da = x(1);
dr = x(2);

disp(['Aileron deflection: ', num2str(rad2deg(da)), ' deg']);
disp(['Rudder deflection: ', num2str(rad2deg(dr)), ' deg']);


GUI = false;
if GUI == 1

Control_GUI

end

% Save data
% save('2gSideslipTurn.mat','U_filter')

