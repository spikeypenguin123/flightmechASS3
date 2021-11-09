% AERO3560 Assignment 3
% Authors: 480373906
% Input: V (m/s), h (m), CG Pos (str), gamma (rads) (is usually zero, can leave
% empty if you dont want to put it in will assume is zero), tolerance
% (default 10^-8), iterMax (defasult is 1000)
% Output: trim structure with alpha, del_t and del_e all in radians

function aircraft = TrimTest2(aircraft, V)
    h = 1000/3.281;
    
    % Check if the asshole using my function put in gamma
    if ~exist('gamma','var'), gamma = 0; end
    
    % check if tolerances have been input
    if ~exist('tol','var'), tol = 10e-8; end
    
    % check if tolerances have been input
    if ~exist('iterMax','var'), iterMax = 1000; end
   
    
    % Grab some constants out hehehehe
    g = aircraft.inertial.g;
    
    % grab the flow properties like a cool man
    [rho,Q] = FlowProperties(aircraft,V);

    % Get ready for initial guess
    CL = aircraft.inertial.m * g / (Q * aircraft.geo.S);

    % define some initial guesses
    alpha_0 = (CL - aircraft.aero.CLo)/aircraft.aero.CLa;
    del_T0  = 0.5;
    del_e0  = 0.01;
    
    % Define error for loooooooop
    error = 1;
    
    % Define x bar
    xBar = [alpha_0;del_T0;del_e0];
    
    % Define Eul
    eul = [0;0;0];
    
    % Define a perturbance
    perturb = 10e-8;
    
    % Lets set up out loop shall we? - loops through until errror is less
    % than tolerance
    iter = 1;
    
    % define an initial guess for alphadothat
    alphaDotHat = 0;
    
    while error > tol && iter < iterMax
        
        % Define velocity parts of x
        x(1,1) = V * cos(xBar(1));
        x(3,1) = V * sin(xBar(1));
        
        % Define parts of u
        u = zeros(4,1);
        u(1) = xBar(2);
        u(2)= xBar(3);
        
        % Lets calculate our euler pitch angle
        theta = xBar(1) + gamma;
        eul(2) = theta;
        
        % Get euler angles into quaternions
        q = e2q(eul);
        
        % Continue to define our x vector
        x(7:10) = q;
        x(11:13) = [0,0,-h];
        
        % Dont actually know if this is the way to do it but ah well
        xDot = StateRates(aircraft,x,u,[alphaDotHat,0])
        xDot(11:13) = [];
        x(11:13) = [];
        
        for i = 1:length(xBar)
            
            % Set initialisation within this for loop
            x_Pet = x;
            u_Pet = u;
            xDot_Pet = [];
            
            % Check to see what we are peturbing
           switch i
               
               % perturbing the alpha - changes the velocties
               case 1
          
                   x_Pet(1) = V * cos(xBar(1) + perturb);
                   x_Pet(3) = V * sin(xBar(1) + perturb);
                   
               % perturbing the del_T
               case 2
                   
                   u_Pet(1) = u(1) + perturb;
               
               % Peturbing the del_e
               case 3
                   
                   u_Pet(2) = u(2) + perturb;
           end
            
            % Obtain rates now
            x_Pet(11:13) = [0,0,-h];
            xDot_Pet = StateRates(aircraft,x_Pet,u_Pet,[alphaDotHat,0])
            xDot_Pet(11:13) = [];
            
            % Calculate big boy J - will need to extract stuff
            J_big = (xDot_Pet - xDot)/perturb;
            
            % Build the jabcoian for our specific case
            J(1,i) = J_big(1);
            J(2,i) = J_big(3);
            J(3,i) = J_big(5);            
        end
        J
        % Define xHat_dot udot ; wdot ; qdot
        xHatDot = [xDot(1); xDot(3); xDot(5)];
        
        % Calculate new xBar
        xBar_new = xBar - inv(J)\xHatDot;
        
        % define the error of this new xBar
        error = sum(abs(xBar_new - xBar));
   
        % redefine x bar
        xBar = xBar_new;
        
        % we have now defined our new x vector - we continue thru loop
        % untill stuff has converged now - calculate error
        iter = iter + 1;
    end
    
    % check to see if we broke loop bc of max iterations reached
    if iter >= iterMax
        warning('Max Iterations Reached - Didn''t converge')
    end

    
    aircraft.controls.delta_T = xBar(2);
    aircraft.controls.delta_e = xBar(3);
end