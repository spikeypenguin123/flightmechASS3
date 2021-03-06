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


function aircraft = Trim(aircraft)
    % Get the state variables into a single vector
    X0 = PullState(aircraft);

    % Control Limits (radians): [dT, de, da, dr]
    ControlMin = aircraft.control_limits.Lower;
    ControlMax = aircraft.control_limits.Upper;

    % Useful Parameters
    S = aircraft.geo.S;
    m = aircraft.inertial.m;                        % Mass (kg)
    g = aircraft.inertial.g;                        % Gravity (m/s^2)
%     altitude = -X0(13);                           % Altitude, -z_e (m)
    [~, ~, V] = AeroAngles(X0);                     % Velocity (m/s)
    [~, Q] = FlowProperties(aircraft, V);           % Density (kg/m^3) and Dynamic Pressure (kPa)
    CL0 = aircraft.aero.CLo;                        % Zero angle of attack lift coefficient
    CLa = aircraft.aero.CLa;                        % dCL/dalpha
    gamma = 0;                                      % climb angle (radians)
    % Estimate CL (lift coefficient)
    CL = m*g/Q/S;
    
    % Guess initial alpha and beta (these guesses were defined in Lec )
    alpha0 = 0; % (CL-CL0)/CLa;  % Estimate for current AoA (rad)
    dT0 = 0.1;              % Initial Thrust    (Newtons)
    de0 = 0.1;              % Initial Elevator  (rad)
    da0 = 0;                % Initial Aileron   (rad)
    dr0 = 0;                % Initial Rudder    (rad)
    U0 = [dT0;de0;da0;dr0]; % Initial Control Vector
    
    tol = 10e-8;        % Error tolerance
    err = 1;            % Intitial Error
    convergence = false;% check for convergence
    maxIter = 1000;     % Number of iterations
    n = 1;              % Intitialise Iteration counter
    delta = 10^-3;      % Perturbation Size
    
    % Perturbation Vector: (Only perturb alpha, Throttle and Elevator)
    xbar0 = [alpha0;U0(1);U0(2)];
    udwdqd = [1,3,5];           % element locations of udot, wdot and qdot in the X state vector
    
    % Initialise the Jacobian
    J = zeros(length(xbar0));
    
    % Newton-Ralphson Numerical Solver
    while ~convergence
        
        % Calculate state for current trim values (set pitch to perturbed
        % alpha) (slide 19 Week 9B)
        att_eul = q2e(X0(7:10));	% Convert state quats to euler angles
        att_eul(2) = xbar0(1) + gamma;       % Set perturbed AoA to the pitch
        X0(7:10) = e2q(att_eul);    % Convert mod'fd attitude to quat
        X0(7:10) = X0(7:10)/norm(X0(7:10));
                
        % State Rate Vector
        Xd0 = TrimRates(X0,U0,aircraft);

        % Non-linear Function f(x) = [udot;wdot;qdot]
        fX = [Xd0(1);Xd0(3);Xd0(5)];
        
        %% Forward difference
        for i = 1:3
            % Initialisation fo the updated state vector 
            X = X0;
            U = U0;

            if i == 1
                % Perturb alpha
                alphaPert = xbar0(1) + delta;
                X(1) = V*cos(alphaPert);            % u: x-vel update
                X(3) = V*sin(alphaPert);            % w: z-vel update
            else 
                % Perturb throttle and elevator
                U(i-1) = xbar0(i) + delta;
            end
            
%             % Update state rates 
             Xd = TrimRates(X,U,aircraft);
            
            % Update Jacobian
            J(:,i) = (Xd(udwdqd) - fX)./delta;
%             J(:,i) = (Xd(udwdqd) - Xd0(udwdqd))/delta;
        end
        
        % Newton-Ralphson Solver: Calculate next trim (xbar) values
        xbar = xbar0 - inv(J)*fX;
        J
        xbar0
        fX
        xbar
        Jinv = inv(J)
        Jdet = det(J)
        % Calculate error
        errV = xbar - xbar0;
        err = max(errV);
        xbar0 = xbar;   % Update xbar for next loop
                
        % Update Control (U0) Vector
        U0(1) = xbar0(2);   % Updated perturbed throttle --> save to Control U0
        U0(2) = xbar0(3);   % Updated perturbed elevator --> save to Control U0
        
        % Update State (X0) Vector
        X0(1) = V*cos(xbar(1));     % Update perturbed x-velocity --> save to State X0
        X0(3) = V*sin(xbar(1));     % Update perturbed z-velocity --> save to State X0
        
        % Ensure compliance with the max and min of controls 
        if  any(U0 < ControlMin)
            disp("A control Input has dropped below its minimum value");
        elseif any(U0 > ControlMax)
            disp("A control Input has risen above its maximum value");
        end

        % Has the Trim Solver convereged to solution?
        if max(err) < tol
            convergence = true;
        end
        
        % Has the maximum number of iterations been completed
        if n < maxIter
            disp("Maximum number of iterations reached in Trim.m");
            break
        end
        n = n+1;         % Increase iteration number
    end    
    
    % Set the Trimmed State and Control to the aircraft struct
    aircraft = PushState(X0,U0,aircraft);
end