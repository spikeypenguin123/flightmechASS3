% Trim.m determine the controls and states for equilibrium steady level flight
% Inputs the current state vector 
% x = [uvw,pqr,q0q1q2q3,xyz_e]

% Outputs the trim tab setting angles in radians 
% u = [dT, de, da, dr]'

function control = Trim(x)
    u = x(1,:);
    v = x(2,:);
    w = x(3,:);
    p = x(4,:);
    q = x(5,:);
    r = x(6,:);
    q0 = x(7,:);
    q1 = x(8,:);
    q2 = x(9,:);
    q3 = x(10,:);
    xe = x(11,:);
    ye = x(12,:);
    ze = x(13,:);
   
    % Guess initial alpha and beta
%     alpha = atan(w,u);
%     beta = asin(v,u);
    
    alpha0 = (CL-CL0)/CLa;
    dT0 = 0.5;
    de0 = 0;
    
    % Set the error tolerance and number of iterations
    tol = 10e-8;
    err = 1;
    maxIter = 1000;
    n = 1;
    
    while err > tol && n < maxIter
        % Calculate state for current trim values
        
        % Calculate state rates for current state and trim values

    % Perturb alpha
        % Recalculate state for current trim values with alpha perturbation
        
        % Recalculate state rates for alpha-perturbed state and trim values
        
    % Perturb throttle
        % Follow same steps as for alpha
    
    % Perturb elevator
        % Follow same steps as for alpha
        
    % Calculate 3x3 Jacobian using perturbed state rates
%     J = 
    
    % Calculate next trim values with Newton iteration
    
    % Calculate error
    err = x(:)
    % Increase iteration number
    n = n+1; 
    end
end