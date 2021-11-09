function aircraft = TrimTest(aircraft, X0)

    % Extract aircraft parameters
    m           = aircraft.Inertial.m;
    g           = aircraft.Inertial.g;
    S           = aircraft.Geo.S;
    CLa         = aircraft.Aero.CLa;
    CLo         = aircraft.Aero.CLo;
    control_min = aircraft.ControlLimits.Lower;
    control_max = aircraft.ControlLimits.Upper;
    
    % Determine aircraft aerodynamic angles and airspeed
    V = aeroangles(X0);
    
    % Determine the flow properties of the aircraft
    [~, Q] = flowproperties(X0, V);
    
    % Estimate the lift coefficient
    CL = m*g/(Q*S);
    
    % Make initial estimates of the inputs
    alpha0 = (CL - CLo)/CLa;
    delta_t0 = 0.5;
    delta_e0 = 0;
    delta_a0 = 0;
    delta_r0 = 0;
    U0 = [delta_t0; delta_e0; delta_a0; delta_r0];

    % Define indices in the state and state rate vector, to be
    % trimmed, udot, wdot, qdot, and w. The rate of change of which should be zero
    % after trimming
    iTrim = [1 3 5];
    
    % Preallocate memory
    J = zeros(length(iTrim));
    
    % Define perturbation increment
    delta = [1e-6; 1e-6; 1e-6];
    
    % Define the xbar vector, i.e. the values to be perturbed
    x_bar = [alpha0; U0(1); U0(2)];
    
    % Initialise convergance boolean and tolerance
    converged = false;
    tol = 1e-9;
    iterLim = 500;
    iterCount = 1;
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while ~converged      
        
%         % Determine the aircraft pitch
%         euler_att = quat2euler(X0(7:10));
%         euler_att(2) = x_bar(1);
%         X0(7:10) = euler2quat(euler_att);
%         
%         % Normalise the quaternion
%         X0(7:10) = X0(7:10)/norm(X0(7:10));
          
        % Determine the state rate vector
        [Xdot] = getstaterates(aircraft, X0, U0);
        fx_bar = Xdot(iTrim);

        % Perturb the variables to get the Jacobian matrix
        for k = 1:length(x_bar)
            
            % Initialise the state and input vector to be trimmed
            X_new = X0;
            U_new = U0;
            
            % For the perturbation of alpha
            if k == 1
                
                % Perturbation of alpha, which affects u and w in the
                % state vector
                X_new(1) = V*cos(x_bar(k) + delta(k));
                X_new(3) = V*sin(x_bar(k) + delta(k));
               
            % For the perturbations of inputs
            else
                
                % Perturbation of the input vector, delta_t and
                % delta_e
                U_new(k-1) = x_bar(k) + delta(k);
            end
            
            % Determine the state rate vector for the perturbed state
            % and input vectors
            [Xdot_new] = getstaterates(aircraft, X_new, U_new);

            % Place in the 'k' column of the Jacobian matrix
            J(:, k) = (Xdot_new(iTrim) - Xdot(iTrim))./(delta(k));
        end
        
        % Update the x_bar vector
        x_bar_new = x_bar - J\fx_bar;
        
        % Determine error
        error = abs((x_bar_new - x_bar)./delta);
        
        % Check if convergance condition is satisfied
        if max(error) < tol
            converged = true;
        end
        
        % Check if iteration limit is reached
        if iterCount > iterLim
            warning('Trim iteration limit reached')
            break
        end
        
        % Update the x_bar vector
        x_bar = x_bar_new;
        
        % Update the state and input vectors
        X0(1) = V*cos(x_bar(1));
        X0(3) = V*sin(x_bar(1));
        U0(1) = x_bar(2);
        U0(2) = x_bar(3);
        
        % Check if exceeding control limits
        if any(U0 > control_max) || any(U0 < control_min)
            
            % Return error for exceeding control limit
            error('Exceeding control limits')
        end
        
        % Incriment iteration count
        iterCount = iterCount + 1;
    end
    
    % Save the final state and input vectors as trimmed vectors
    X_trimmed = X0;
    U_trimmed = U0;    
    
    
    
end