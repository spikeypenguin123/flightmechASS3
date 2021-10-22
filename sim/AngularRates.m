function AngularRates = AngularRates(X,Xd)

% INPUT: Body velcoities and accelerations
% OUTPUT: Angular Rates for angle of attack and sideslip

%% Computing Angular Rates
% Week 9A Slide 7

u = X(1);
v = X(2);
w = X(3);
udot = Xd(1);
vdot = Xd(2);
wdot = Xd(3);

% Magnitude of velocities
V = sqrt(u^2 + v^2 + w^2);

% Sideslip and angle of attack calculations
alpha = atan2(w,u);
beta = asin2(v,V);

% Sideslip and angle of attack angular rates
AngularRates(1) = (wdot/V)*sec(alpha)*sec(beta);    % alphad: Angle of attack rate 
AngularRates(2) = (vdot/V)*sec(beta);                       % betad: Sideslip rate
    

end