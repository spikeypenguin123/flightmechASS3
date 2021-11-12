function angular_rates = AngularRates(X,Xd)

% INPUT: Body velcoities and accelerations
% OUTPUT: Angular Rates for angle of attack and sideslip

%% Computing Angular Rates
% Week 9A Slide 7

u = X(1);
v = X(2);
w = X(3);
vdot = Xd(2);
wdot = Xd(3);

% Magnitude of velocities
V = sqrt(u^2 + v^2 + w^2);

% Sideslip and angle of attack calculations
alpha = atan2(w,u);
beta = asin(v/V);

% Sideslip and angle of attack angular rates
angular_rates(1) = (wdot/V)*sec(alpha)*sec(beta);    % alphad: Angle of attack rate 
angular_rates(2) = (vdot/V)*sec(beta);               % betad: Sideslip rate
    

end