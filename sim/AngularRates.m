function [alphadot, betadot] = AngularRates([States],[Rates])

% INPUT: Body velcoities and accelerations
% OUTPUT: Angular Rates for angle of attack and sideslip

%% Computing Angular Rates
% Week 9A Slide 7

u = States(1);
v = States(2);
w = States(3);
udot = Rates(1);
vdot = Rates(2);
wdot = Rates(3);

% Magnitude of velocities
V = sqrt(u^2 + v^2 + w^2);

% Sideslip and angle of attack calculations
alpha = atan(w/u);
beta = asin(v/V);

% Sideslip and angle of attack angular rates
alphadot = (wdot/V)*sec(alpha)*sec(beta);
betadot = (vdot/V)*sec(beta);
    

end