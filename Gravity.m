% Computes the force of gravity in the body axes 

% Inputs:
% g: gravity (m/s^2)
% m: mass (kg)
% q: 4x1 quaternions (rad) 

% Output: 
% G_body: 3x1 Force (N)
function G_body = Gravity(g,q,m)
    G_body = DCM(q)*[0;0;m*g];
end
