function Xd = TrimRates(X,U,aircraft)
    % Calculate the state rates for the Trim.m function
    % X: Current State Vector
    % U: Control Vector
    % Data: Flight Data
    
    Xd = zeros(13,1);   % Initialise xdot for speed (alphadot and betadot)

    for i = 1:3
        Xd = StateRates(X,Xd,U,aircraft);
    end
end