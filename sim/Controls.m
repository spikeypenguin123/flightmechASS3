function [delta_T, delta_e, delta_a, delta_r] = Controls(flight_plan, t, delta_T, delta_e, delta_a, delta_r)
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
        % TODO: cases 4-8
        otherwise
            disp("No flight plan found.")
    end
    
end

