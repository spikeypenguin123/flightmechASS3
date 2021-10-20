% Trim.m determine the controls and states required for equilibrium steady level flight
% Once the equilibrium condition is specified, we need to determine the
% combination of steady control inputs that are required in order to
% maintain that equilibrium --> aka trimming the simulation! 

% 

% Needs to calculate u for which xd = 0



% Inputs the current state vector 
% x = [uvw,pqr,q0q1q2q3,xyz_e]

% Outputs the trim tab setting angles in radians 
% u = [dT (Thrust), de (elevator), da (aileron), dr (rudder)]'


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
   
    % Guess initial alpha and beta (these guesses were defined in Lec )
    alpha0 = (CL-CL0)/CLa;
    dT0 = 0.5;      % Initial Thrust
    de0 = 0;        % Initial Elevator
    da0 = 0;        % Initial Aileron
    dr0 = 0;        % Initial Rudder
    control = [dT0;de0;da0;dr0]; 
    
    % Set the error tolerance and number of iterations
    tol = 10e-8;
    err = 1;
    maxIter = 1000;
    n = 1;
    
    while err > tol && n < maxIter
        % Calculate state for current trim values
        
        % Calculate state rates for current state and trim values
        % is this the xdot? in the jacobian calc? 
        
            
    % Perturb alpha +delta and -delta
        % Recalculate state for current trim values with alpha perturbation
        
        % Recalculate state rates for alpha-perturbed state and trim values
        
    % Perturb throttle +delta and -delta
    J(:,1) = (fx_k2p - fx_k2m)/(2*deltax);

    
    % Perturb elevator +delta and -delta
    
    % Assign Elevator deflection Dependance to associated column of Jacobian
    J(:,2) = (fx_k2p - fx_k2m)/(2*deltax);
    
    % Calculate 3x3 Jacobian using perturbed state rates
%     J = 
    
    % Calculate next trim values with Newton iteration
    
    
    % Calculate error
    err = control - control_k;
    % Increase iteration number
    n = n+1; 
    end
end