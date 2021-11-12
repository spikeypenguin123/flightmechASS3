% AeroAngles.m

% Inputs the state vector and uses only the first 3 velocity terms
function [alpha, beta, V] = AeroAngles(X)
    
    % Velocity (m/s)
    V = sqrt(X(1).^2 + X(2).^2 + X(3).^2);
    
    % Calculate Angle of Attack, alpha (rad)
    alpha = atan(X(3)/X(1));
    
    % Calcuate Side slip Angle, beta (rad)
    beta = asin(X(2)/V);
end