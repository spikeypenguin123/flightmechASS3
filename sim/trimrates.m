function Xd = TrimRates(X,U,aircraft)
    % Calculate the state rates for the Trim.m function
    % X: Current State Vector
    % U: Control Vector
    % Data: Flight Data
    
    % Intiialise the angular rates
    angularrates0 = [0,0];
    err = 1;
    eps = 10^-8;
    n = 1;
    maxIter = 200;
    
    Xd = zeros(13,1);   % Initialise xdot for speed (alphadot and betadot)

    while eps < err && n < maxIter
        
        % Estimate State Rate
        Xd = StateRates(aircraft, X, U, angularrates0);

        % Recalculate Angular rates
        angularratesnext = AngularRates(X,Xd);
        
        % Error Calculation prior to saving
        err = max(abs((angularratesnext - angularrates0)./angularrates0));
        
        % Save to the old Anglular rate variables
        angularrates0 = angularratesnext;
        
        if n >= maxIter
           error("Max Iteration in Trim Rates"); 
        end
        
        n = n + 1; % increase the counter 
    end
end