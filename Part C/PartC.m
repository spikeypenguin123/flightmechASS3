clc
clear
clear all

% load helper functions from flight sim
addpath('../sim')

% Go through each case
for cg = ["CG1", "CG2"]
    for speed = [100, 180]
        if cg == "CG2" && speed == 180
            % we only do 3 cases so lets call it here
            return
        end
        disp("Case: " + cg + "@" + speed + "kts")

        if cg == "CG1"
            data = aero3560_LoadFlightDataPC9_nominalCG1();
        else
            data = aero3560_LoadFlightDataPC9_CG2();
        end
        
        % alt 1000ft
        alt = 1000*0.3048; %m
        V = speed*0.51444;
        
%         % use atmospheric model from flight sim
%         [rho, Q] = FlowProperties(alt, V, data.Inertial.g);
        rho = 1.225; % Need to use correct density
        
        
        %% Part A
        disp('Part A');
        % Using Dynamic Stability Matrix from Week 10A
        
        
        
        
        disp('------------------------------------');
        
        %% Part B
        disp('Part B');
        
        
        disp('------------------------------------');
        
        %% Part C
        disp('Part C');
        
        
        disp('------------------------------------');
        
        %% Part D
        disp('Part D');
        
        
        disp('------------------------------------');
        
        %% Part E
        disp('Part E');
        
        
        disp('------------------------------------');
        
        %% Part F
        disp('Part F');
        
        
        
        disp('------------------------------------');
        
    end
end


