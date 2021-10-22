% Trim.m determine the controls and states required for equilibrium steady level flight
% Once the equilibrium condition is specified, we need to determine the
% combination of steady control inputs that are required in order to
% maintain that equilibrium --> aka trimming the simulation! 

% Needs to calculate u for which xd = 0

% DO NOT NEED TO TRIM TURNS OF LOOPS SO ONLY FOR STEADY LEVEL FLIGHT
% ONLY perturb ALPHA, THROTTLE, ELEVATOR

% Inputs
% XO = [uvw,pqr,q0q1q2q3,xyz_e]
% aircraft: struct containing aircraft data

% Outputs 
% Xtrimmed = Trimmed state vector (same structure as input)
% Utrimmed = [dT (Thrust), de (elevator), da (aileron), dr (rudder)]'

% Other Variables
% xbar0: The current vector containing the variables requiring perturbation
% xbar: The new vector containing the variables requiring perturbation


function [Xtrimmed, Utrimmed] = Trim(X0, aircraft)
    u = X0(1);    % (m/s)       1
    v = X0(2);    % (m/s)       2
    w = X0(3);    % (m/s)       3
    p = X0(4);    % (rad/s)     4
    q = X0(5);    % (rad/s)     5
    r = X0(6);    % (rad/s)     6
    q0 = X0(7);   % quat w      7
    q1 = X0(8);   % quat x      8
    q2 = X0(9);   % quat y      9
    q3 = X0(10);  % quat z      10
    xe = X0(11);  % (m)         11
    ye = X0(12);  % (m)         12
    ze = X0(13);  % (m)         13
   
    
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

    % Estimate CL (lift coefficient)
    CL = m*g/Q/S;
    
    % Guess initial alpha and beta (these guesses were defined in Lec )
    alpha0 = (CL-CL0)/CLa;  % Estimate for current AoA (rad)
    dT0 = 0.5;              % Initial Thrust    (Newtons)
    de0 = 0;                % Initial Elevator  (rad)
    da0 = 0;                % Initial Aileron   (rad)
    dr0 = 0;                % Initial Rudder    (rad)
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
    
    % Perturbation Vector: (Only perturb alpha, Throttle and Elevator)
    xbar0 = [alpha0;U0(1);U0(2)];
    
    % Initialise the Jacobian
    J = zeros(length(x_bar));

    % Newton-Ralphson Numerical Solver
    while err > tol && n < maxIter
        
        % Calculate state for current trim values (set pitch to perturbed alpha)
        att_eul = q2e(X0(7:10));	% Convert state quats to euler angles
        att_eul(2) = xbar0(1);       % Set perturbed AoA to the pitch
        X0(7:10) = e2q(att_eul);    % Convert mod'fd attitude to quat
        X0(7:10) = X0(7:10)/norm(X0(7:10));
        
        % State Rate Vector
        Xd0 = TrimRates(X0,U0,aircraft);

        % Non-linear Function f(x) = [alphaDot;ThrotDot;ElevDot]
        fX = [Xd0(1);Xd0(3);Xd0(5)];
        
        % Initialisation fo the updated state vector 
        X = X0;
        U = U0;
        
        % Perturb alpha
        % Recalculate state for current trim values with alpha perturbation
        alphaPert = xbar0(1) + delta;
        X(1) = V*cos(alphaPert);            % u: x-vel update
        X(3) = V*sin(alphaPert);            % w: z-vel update
        Xd = TrimRates(X,U,aircraft);       % Update state rates 
        J(:,1) = (Xd - Xd0)./(2.*deltax);   % Jacobian for alpha dependance

        % Perturb throttle
        U(1) = xbar0(2) + delta; 
        Xd = TrimRates(X,U,aircraft);       % Update state rates 
        J(:,3) = (Xd - Xd0)./(2.*deltax);   % Jacobian for throttle dependance
        
        % Perturb elevator
        U(2) = xbar0(3) + delta;
        Xd = TrimRates(X,U,aircraft);       % Update state rates 
        J(:,3) = (Xd - Xd0)./(2.*deltax);   % Jacobian for elevator dependance

        % Newton-Ralphson Solver: Calculate next trim (xbar) values
        xbar = xbar0 - J\fX;

        % Calculate error
        err = abs((xbar - xbar0)./delta);
        xbar0 = xbar;   % Update xbar for next loop
        
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