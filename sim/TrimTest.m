function aircraft = TrimTest(aircraft)
    h = 1000/3.281;
    % Extract aircraft parameters
    m = aircraft.inertial.m;                        % Mass (kg)
    g = aircraft.inertial.g;                        % Gravity (m/s^2)
    S = aircraft.geo.S;
    CL0 = aircraft.aero.CLo;                        % Zero angle of attack lift coefficient
    CLa = aircraft.aero.CLa;                        % dCL/dalpha
    control_min = aircraft.control_limits.Lower;
    control_max = aircraft.control_limits.Upper;
    X0 = PullState(aircraft);

    % Determine aircraft aerodynamic angles and airspeed
    [~, ~, V] = AeroAngles(X0);                     % Velocity (m/s)
    
    % Determine the flow properties of the aircraft
    [~, Q] = FlowProperties(aircraft, V);           % Density (kg/m^3) and Dynamic Pressure (kPa)
    
    % Estimate the lift coefficient
    CL = m*g/(Q*S);
    
    % Make initial estimates of the inputs
    alpha0 = (CL - CL0)/CLa;
    dT0 = 0.1;
    de0 = -0.01;
    da0 = 0;
    dr0 = 0;
    U0 = [dT0; de0; da0; dr0];

    % Define indices in the state and state rate vector, to be
    % trimmed, udot, wdot, qdot, and w. The rate of change of which should be zero
    % after trimming
    iTrim = [1 3 5];
    
    % Preallocate memory
    J = zeros(length(iTrim));
    
    % Define perturbation increment
    delta = 1e-9+zeros(3);
    
    % Define the xbar vector, i.e. the values to be perturbed
    x_bar = [alpha0; U0(1); U0(2)];
    
    % Initialise convergance boolean and tolerance
    converged = false;
    tol = 1e-6;
    iterLim = 1000;
    iterCount = 1;
    
    Xdot = zeros(13);
    
    % Numerical Newton-Ralphson method to solve for control inputs
    while ~converged      
        
        % Determine the aircraft pitch
        euler_att = q2e(X0(7:10));
        euler_att(2) = x_bar(1);
        X0(7:10) = e2q(euler_att);
        
        % Normalise the quaternion
        X0(7:10) = X0(7:10)/norm(X0(7:10));
          
        % Determine the state rate vector
        X0(11:13)=[0;0;-h];
        Xdot = StateRates(aircraft, X0, U0, AngularRates(X0,Xdot) );
        X0(11:13)=[];
        Xdot(11:13)=[];

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
            X_new(11:13) = [0;0;-h];
            Xdot_new = StateRates( aircraft, X_new, U_new, AngularRates(X_new,Xdot));
            Xdot_new(11:13)=[];
            
            % Place in the 'k' column of the Jacobian matrix
            J(:, k) = (Xdot_new(iTrim) - Xdot(iTrim))./(delta(k));
        end
        
        fx_bar = Xdot(iTrim);
        % Update the x_bar vector
        x_bar_new = x_bar - J\fx_bar;
        
        % Determine error
        
        error = sum(abs(x_bar_new - x_bar))
        
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
%         if any(U0 > control_max) || any(U0 < control_min)
%             
%             % Return error for exceeding control limit
%             error('Exceeding control limits')
%         end
        
        % Incriment iteration count
        iterCount = iterCount + 1;
    end
      
    % Set the Trimmed State and Control to the aircraft struct
    aircraft.controls.delta_e = U0(2);
    aircraft.controls.delta_T = U0(1);
end