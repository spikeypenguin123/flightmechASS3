% Trim.m determine the controls and states required for equilibrium steady level flight
% Once the equilibrium condition is specified, we need to determine the
% combination of steady control inputs that are required in order to
% maintain that equilibrium --> aka trimming the simulation! 

% Needs to calculate u for which xd = 0


% Inputs the initial state vector and the Data
% XO = [uvw,pqr,q0q1q2q3,xyz_e]

% Outputs the trim tab setting angles in radians 
% u = [dT (Thrust), de (elevator), da (aileron), dr (rudder)]'


function [Xtrimmed, Utrimmed] = Trim(X0, aircraft)
    u = X0(1,:);    % (m/s)
    v = X0(2,:);    % (m/s)
    w = X0(3,:);    % (m/s)
    p = X0(4,:);    % (rad/s)
    q = X0(5,:);    % (rad/s)
    r = X0(6,:);    % (rad/s)
    q0 = X0(7,:);
    q1 = X0(8,:); 
    q2 = X0(9,:);
    q3 = X0(10,:);
    xe = X0(11,:);  % (m)
    ye = X0(12,:);  % (m)
    ze = X0(13,:);  % (m)
   
    
    % Control Limits (radians): [dT, de, da, dr]
    ControlMin = aircraft.control_limits.Lower;
    ControlMax = aircraft.control_limits.Upper;

    % Useful Parameters
    S = aircraft.geo.S;
    m = aircraft.inertial.m;                        % Mass (kg)
    g = aircraft.inertial.g;                        % Gravity (m/s^2)
    [~, ~, V] = AeroAngles(X0);                     % Velocity (m/s)
    [~, Q] = FlowProperties(altitude, V, g);      % Density (kg/m^3) and Dynamic Pressure (kPa)
    CL0 = aircraft.aero.CLo;                        % Zero angle of attack lift coefficient
    CLa = aircraft.aero.CLa;                        % dCL/dalpha

    % Estimate CL
    CL = m*g/Q/S;
    
    % Guess initial alpha and beta (these guesses were defined in Lec )
    alpha0 = (CL-CL0)/CLa;
    dT0 = 0.5;              % Initial Thrust
    de0 = 0;                % Initial Elevator
    da0 = 0;                % Initial Aileron
    dr0 = 0;                % Initial Rudder
    U0 = [dT0;de0;da0;dr0]; % Initial Control Vector
    
   
    tol = 10e-8;        % Error tolerance
    err = 1;            % Intitial Error
    maxIter = 1000;     % Number of iterations
    n = 1;              % Intitialise Iteration counter
    delta = 10^-5;      % Perturbation Size

    
    % What state variables should be constant at steady state? 
    % Forward velocity constant:    udot    = 0
    % Upwards velocity constant:    wdot    = 0
    % Pitch should be constant:     q       = 0
    
    % State wrt perturbation of alpha
    % use calcrates function which is 
    
    % Perturbation Vector: (Only perturb alpha, Throttle and Elevator)
    xbar = [alpha0;U0(1);U0(2)];
    
    % Initialise the Jacobian
    J = zeros(length(x_bar));

    
    while err > tol && n < maxIter
        
        
        
        % Calculate state for current trim values
        
        
        % Calculate state rates for current state and trim values
        % is this the xdot? in the jacobian calc? 
        
        % State Rate Vector
        Xd = TrimRates(X,U,aircraft);
 
        % Non-linear Function f(x) = [alphaDot;ThrotDot;ElevDot]
        fX = [Xd(1);Xd(3);Xd(5)];
        
    % Perturb alpha
        % Recalculate state for current trim values with alpha perturbation
        
        % Recalculate state rates for alpha-perturbed state and trim values
        
    % Perturb throttle
    J(:,1) = (fx_k2p - fx_k2m)/(2*deltax);

    
    % Perturb elevator
    
    J(:,2) = (fx_k2p - fx_k2m)/(2*deltax);
    
    % Calculate 3x3 Jacobian using perturbed state rates
%    J = ;
    

% DO NOT NEED TO TRIM TURNS OF LOOPS SO ONLY FOR STEADY LEVEL FLIGHT
% ONLY perturb ALPHA, THROTTLE, ELEVATOR


    % Calculate next trim values with Newton iteration
    
    
    % Calculate error
    err = control - control_k;
    
    % Ensure compliance with the max and min of controls 
    if  any(U0 < ControlMin)
        disp("A control Input has dropped below its minimum value");
    else if any(U0 > ControlMax)
        disp("A control Input has risen above its maximum value");
    end
    
    % Increase iteration number
    n = n+1; 
    end
    
    Xtrimmed = X0;
    Utrimmed = U0;
end