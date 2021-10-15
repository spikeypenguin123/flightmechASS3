function [alphadot, betadot] = AngularRates()

% INPUT:
% OUTPUT:

%% Computing Angular Rates
    
% Ignoring secondary aerodynamic effects, propulsive effects and trim
% effects - using linearised forms
% Expressions for alphadot and betadot can be found on Week 8B
alphadot = (Cl - Cl0 - Clalpha*alpha - Clq*qbar - Clde*de)/Clalphadot;
alphadot = (Cm - Cm0 - Cmalpha*alpha - Cmq*qbbar - Cmde*de)/Cmalphadot;
    

end