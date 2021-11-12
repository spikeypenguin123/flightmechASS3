function Xdot = TrimStateRates(X, U, aircraft)
  
    % Initial guess
    alpha_dot_old = 0;
    beta_dot_old = 0;

    % Errors
    error = 1;
    tolerance = 1e-9;
    
    % Set iteration limit
    iterLim = 100;
    iterCount = 0;
    
    % Iterate until angle of attack and sideslip rates converge
    while error > tolerance
        
        % Concatenate previous iteration angle of attack/sideslip rates to
        % be passed to 'staterates' function
        angular_rates = [alpha_dot_old, beta_dot_old];

        % Estimate state rates using angle of attack and sideslip rates
        Xdot = StateRates(aircraft, X, U, angular_rates);
        
        % Calculate angular rates
        angular_rates = AngularRates(X,Xdot);
        
        alpha_dot = angular_rates(1);
        beta_dot = angular_rates(2);
        
        % Calculate errors in angle of attack and sideslip rates
        error_alpha_dot = abs((alpha_dot - alpha_dot_old)/alpha_dot_old);
        error_beta_dot = abs((beta_dot - beta_dot_old)/beta_dot_old);
        error = max([error_alpha_dot error_beta_dot]);
    
        % Store current angle of attack/sideslip rates for next iteration
        alpha_dot_old = alpha_dot;
        beta_dot_old = beta_dot;
    
        % Break with warning if iteration limit is reached
        if iterCount > iterLim
            warning('Reached iteration limit for alpha and beta rates!');
            break
        end
    
        % Increment iteration counter
        iterCount = iterCount + 1;
    end  
end