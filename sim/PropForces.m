function  [Pmax, Thrust] = PropForces(rho,V_T, delta_T)
  
% INPUT:
% OUTPUT:

%% Computing Propulsion Fores

rho_SL = 1.225;
sigma = rho/rho_SL;
eta = Prop.eta;
Pmax_SL = Prop.P_max;

Pmax = Pmax_SL*(1.1324*sigma - 0.1324); % Max Power
Thrust = ((eta*Pmax)/V_T)*delta_T; % Thrust


end