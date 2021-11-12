function  [Pmax, Thrust] = PropForces(rho,u,delta_T,Prop)

% Computing Propulsion Fores

rho_SL = 1.225;
sigma = rho/rho_SL;
eta = Prop.eta; % Data from flight condition
Pmax_SL = Prop.P_max; % Data from flight condition

Pmax = Pmax_SL*(1.1324*sigma - 0.1324); % Max Power
Thrust = (eta*Pmax*delta_T)/u; % Thrust


end