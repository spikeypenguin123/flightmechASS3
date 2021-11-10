function [delta_T, delta_e, delta_a, delta_r] = Controls(flight_plan, t, i, delta_T, delta_e, delta_a, delta_r)
    % inputs: flight plan, time
    % flight plan is an int corresponding to the case studied (1-8)
    % output: predefined control setting based on flight plan (5x1)
    % controls should be estimated using the Control_GUI if possible
    
    % note that if a control is not set, it will remain as the trimmed
    % control
    
    switch flight_plan
        % cases 1-3 are simple impulses. wait 1 sec before applying.
        case 1
            if t >= 1 && t <= 1.5
                delta_e = deg2rad(5)+delta_e;
            end
        case 2
            if t >= 1 && t <= 1.5
                delta_a = deg2rad(5)+delta_a;
            end
        case 3
            if t >= 1 && t <= 1.5
                delta_r = deg2rad(5)+delta_r;
            end
        case 4
            if t <= 15.6
                load('Loop.mat');
                c = controls(:,i);
                delta_e = c(1) + delta_e;
            end
        case 5
            load('2gSideslipTurn.mat');
            c = deg2rad(U_filter(:,i));
            
            delta_T = c(1)+delta_T;
            delta_e = c(2)+delta_e;
            delta_a = c(3)+delta_a;
            delta_r = c(4)+delta_r;
        case 6
            load('StepOnBall.mat'); % steady heading sideslip requires the pilot to "step on the ball"
            c = deg2rad(U_filter(:,i));
            
            delta_T = c(1)+delta_T;
            delta_e = c(2)+delta_e;
            delta_a = c(3)+delta_a;
            delta_r = c(4)+delta_r;
        case 7
            load('DelayedRoll.mat');
            
            c = deg2rad(U_filter(:,i));
            if abs(c(1)) > 0.01
                delta_T = c(1);
            end
            if abs(c(2)) > 0.01
                delta_e = max(c(2), delta_e);
            end
            delta_a = c(3);
            delta_r = c(4);
        case 8
            if t <= 5
                load('BarrelRollStage1.mat');
                c = optimresults.x(:,i);
                delta_T = c(1)+delta_T;
                delta_e = c(2)+delta_e;
                delta_a = deg2rad(7.5)+delta_a;
                delta_r = c(4)+delta_r;
            end
            
            
        % TODO: cases 4-8
        otherwise
            disp("No flight plan found.")
    end
    
end

