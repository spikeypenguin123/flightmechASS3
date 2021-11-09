function aircraft = Trim(aircraft)

    % Extract aircraft parameters
    m = aircraft.inertial.m;                        % Mass (kg)
    g = aircraft.inertial.g;                        % Gravity (m/s^2)
    S = aircraft.geo.S;                             % wing area (m^2)
    CL0 = aircraft.aero.CLo;                        % Zero angle of attack lift coefficient
    CLa = aircraft.aero.CLa;                        % dCL/dalpha
    control_min = aircraft.control_limits.Lower;    % Control surface min limit
    control_max = aircraft.control_limits.Upper;    % Control surface max limit
    X0 = PullState(aircraft);	% Pull state variables into a 13x1 vector
    
    % Indexs for u, w and q in the state vector
    uwq = [1 3 5];
    
    % Aircraft velocity (m/s)
    V = sqrt(X0(1).^2 + X0(2).^2 + X0(3).^2);
    
    % Dynamic Pressure (kPa)
    [~, Q] = FlowProperties(aircraft, V);
    
    % Lift coefficient
    CL = m*g/(Q*S);
    
    % Set an initial estimate for control vector and the AoA
    alpha0 = (CL - CL0)/CLa;
    dT0 = 0.5;
    de0 = 0;
    da0 = 0;
    dr0 = 0;
    % Create the control vector
    U0 = [dT0; de0; da0; dr0]; 

    % Make Empty Jacobian for speed
    J = zeros(3);
    
    % Define perturbation increment
    delta = 1e-6;
    
    % Define the xbar vector, i.e. the values to be perturbed
    x_bar = [alpha0; U0(1); U0(2)];
    
    % Initialise convergance boolean and tolerance
    converged = false;
    tol = 1e-10;
    maxIter = 500;
    n = 1;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while ~converged      
        
        % Determine the aircraft pitch
        euler_att = q2e(X0(7:10));
        euler_att(2) = x_bar(1);
        X0(7:10) = e2q(euler_att);
        
        % Normalise the quaternion
        X0(7:10) = X0(7:10)/norm(X0(7:10));
          
        % Determine the state rate vector
        Xdot = TrimStateRates(X0, U0, aircraft);
        fx_bar = Xdot(uwq);

        % Perturb the variables to get the Jacobian matrix
        for k = 1:length(x_bar)
            
            % Initialise the state and input vector to be trimmed
            Xnew = X0;
            Unew = U0;
            
            % For the perturbation of alpha
            if k == 1
                
                % Perturbation of alpha, which affects u and w in the
                % state vector
                Xnew(1) = V*cos(x_bar(k) + delta);
                Xnew(3) = V*sin(x_bar(k) + delta);
               
            % For the perturbations of inputs
            else
                
                % Perturbation of the input vector, delta_t and
                % delta_e
                Unew(k-1) = x_bar(k) + delta;
            end
            
            % State rate vector for the perturbed state and input vectors
            Xdot_new = TrimStateRates(Xnew, Unew, aircraft);

            % Build Jacobian
            J(:, k) = (Xdot_new(uwq) - Xdot(uwq))./(delta);
        end
        
        % Update x_bar
        x_bar_new = x_bar - J\fx_bar;
        
        % Determine error
        error = abs((x_bar_new - x_bar)./delta);
        
        % Check if solution has converged
        if max(error) < tol
            converged = true;
        end
        

        
        % Update the x_bar vector
        x_bar = x_bar_new;
        
        % Update the state and input vectors
        X0(1) = V*cos(x_bar(1));
        X0(3) = V*sin(x_bar(1));
        U0(1) = x_bar(2);
        U0(2) = x_bar(3);
        
%       Aircraft control limits
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