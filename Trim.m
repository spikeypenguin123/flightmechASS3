function aircraft = Trim(aircraft)

    % Aircraft Data 
    X0 = PullState(aircraft);	% Pull state variables into a 13x1 vector
    uwq = [1, 3, 5];  % Indexs for u, w and q in the state vector

    % Aircraft Data 
    S = aircraft.geo.S;                             % wing area (m^2)
    m = aircraft.inertial.m;                        % Mass (kg)
    g = aircraft.inertial.g;                        % Gravity (m/s^2)
    CL0 = aircraft.aero.CLo;                        % Zero angle of attack lift coefficient
    CLa = aircraft.aero.CLa;                        % dCL/dalpha
    control_min = aircraft.control_limits.Lower;    % Control surface min limit
    control_max = aircraft.control_limits.Upper;    % Control surface max limit

    % Aircraft velocity (m/s)
    V = sqrt(X0(1).^2 + X0(2).^2 + X0(3).^2);
    
    % Dynamic Pressure (kPa)
    [~, Q] = FlowProperties(aircraft, V);
    
    % Lift coefficient
    CL = m*g/(Q*S);

    alpha0 = (CL - CL0)/CLa;    % Estimate for current AoA (rad)
    dT0 = 0.5;                  % Initial Thrust    (Newtons)
    de0 = 0;                    % Initial Elevator  (rad)
    da0 = 0;                    % Initial Aileron   (rad)
    dr0 = 0;                    % Initial Rudder    (rad)
    U0 = [dT0;de0;da0;dr0];     % Initial Control Vector
    
    % Define the xbar vector, i.e. the values to be perturbed
    xbar0 = [alpha0; U0(1); U0(2)];
    
    % Initialise convergance boolean and tolerance
    tol = 10e-10;           % Error tolerance
    err = 1;                % Intitialise Error
    maxIter = 500;          % Number of iterations
    n = 1;                  % Intitialise Iteration counter
    delta = 1e-7;           % Perturbation Size

    % Make Empty Jacobian for speed
    J = zeros(3);
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while max(err) >= tol      % Run until the error is sufficiently small and solution has convereged
        
        % Setting pitch to AoA (slide 19 Week 9B)
        euler_att = q2e(X0(7:10));
        euler_att(2) = xbar0(1);
        X0(7:10) = e2q(euler_att);
        
        % Normalise the quaternion
        X0(7:10) = X0(7:10)/norm(X0(7:10));
          
        % State Rate
        Xd0 = TrimStateRates(X0, U0, aircraft);

        % Non-linear Function f(x) = [udot;wdot;qdot]
        fxbar = Xd0(uwq);

        % Perturb the variables to get the Jacobian matrix
        for i = 1:3
            
            % Initialise the state and input vector to be trimmed
            X = X0;
            U = U0;
            
            % For the perturbation of alpha
            if i == 1
                alphaPert = xbar0(i) + delta;   % Perturb alpha
                X(1) = V*cos(alphaPert);        % u: x-vel update
                X(3) = V*sin(alphaPert);        % w: z-vel update
            else
                % Perturb the Throttle and Elevator
                U(i-1) = xbar0(i) + delta;
            end
            
            % Update state rates 
            Xd = TrimStateRates(X, U, aircraft);

            % Build Jacobian
            J(:, i) = (Xd(uwq) - Xd0(uwq))./(delta);
        end
        
        % Update x_bar
        xbar = xbar0 - J\fxbar;
        
        % Determine error
        err = abs((xbar - xbar0)./delta);
        
        % Update the x_bar vector storing alpha, dT and de
        xbar0 = xbar;
        
        % Control Vector Update 
        U0(1) = xbar0(2);
        U0(2) = xbar0(3);
        
        % State Vector Update
        X0(1) = V*cos(xbar0(1));
        X0(3) = V*sin(xbar0(1));

        % Aircraft control limits
        if any(U0 > control_max) || any(U0 < control_min)
            disp('WARNING: THE THROTTLE OR ELEVATOR HAVE EXCEEDED LIMITS!')
        end
        
        % Check for the iteration limit
        if n > maxIter
            disp('WARNING: ITERATION LIMIT IN TRIM')
        end
        
        % Incriment iteration count
        n = n + 1;
    end
      
    % Set the Trimmed State and Control to the aircraft struct
    aircraft = PushState(X0,U0,aircraft);
end