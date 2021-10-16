clear
close all
clc

% load helper functions from flight sim
addpath('../sim')

% Go through each case
for cg = ["CG1", "CG2"]
    for speed = [100, 180]
        disp("Case: " + cg + "@" + speed + "kts")

        if cg == "CG1"
            data = aero3560_LoadFlightDataPC9_nominalCG1();
        else
            data = aero3560_LoadFlightDataPC9_CG2();
        end
        
        % alt 1000ft
        alt = 1000*0.3048; %m
        V = speed*0.51444;
        
        % use atmospheric model from flight sim
        [rho, Q] = FlowProperties(alt, V, data.Inertial.g);

        %% Part a: compute equilibrium conditions
        disp("Part A")
        
        dpsi = deg2rad(360)/30; % yaw rate
        
        % we use iteration because small angle assumption is a bit too dicey here
        
        % initial guesses:
        r = pi/4;       
        C_L = data.Inertial.m*data.Inertial.g/(Q*data.Geo.S);
        F_c = data.Inertial.m*V*r/data.Geo.S/Q;
        phi = F_c/C_L;
        
        
        eps = 1e-5;
        d = 1e5;
        while d > eps
            
            % recalc C_L
            C_L=data.Inertial.m*data.Inertial.g/(Q*data.Geo.S*cos(phi));
            
            r = dpsi*cos(phi);
            
            % get a better estimate for bank angle (rearrange lecture 7A
            % slide 19 with C_Y=0)
            phi_next = F_c/C_L;

            % nondimensional centripetal force
            F_c = data.Inertial.m*((V*r)/(data.Geo.S*Q));
            
            d = abs(phi-phi_next);
            
            % make sure it converges in the right direction
            phi = abs(phi_next);
        end
        
        n_z = C_L*Q*data.Geo.S/data.Inertial.m/data.Inertial.g;
        
        % Note all these values are independent of CG
        disp("Coefficient of Lift: "+C_L)
        disp("Loading factor: "+n_z)
        disp("Bank angle: "+rad2deg(phi)+"deg")
        disp("Yaw rate: "+rad2deg(dpsi)+"deg/s")
        disp("Body rate r: "+rad2deg(r)+"deg/s")

        
        %% part b: body rates, trim conditions
        disp("Part B")
        p = 0;
        q = dpsi*sin(phi);
        
        % non-dimensionalised roll rate r
        r_bar = r*data.Geo.b/(2*V);
        
        delta_a = (data.Aero.Cldr*data.Aero.Cnr-data.Aero.Cndr*data.Aero.Clr)/(data.Aero.Cndr*data.Aero.Clda-data.Aero.Cldr*data.Aero.Cnda)*r_bar;
        delta_r = (data.Aero.Cnda*data.Aero.Clr-data.Aero.Clda*data.Aero.Cnr)/(data.Aero.Cndr*data.Aero.Clda-data.Aero.Cldr*data.Aero.Cnda)*r_bar;
    
        disp("Body rates (p,q,r): (" + rad2deg(p) + "," + rad2deg(q) + "," + rad2deg(r) + ")deg/s")
        disp("Aileron deflection: " + rad2deg(delta_a) + "deg")
        disp("Rudder deflection: " + rad2deg(delta_r) + "deg")
        
        %% part c-d: aileron-only turn and stability
        disp("Part C")
        
        % Initial guesses for iteration
        delta_a_trim = (data.Aero.Clb*data.Aero.Cnr-data.Aero.Cnb*data.Aero.Clr)/(data.Aero.Clda*(data.Aero.Cnb*(1-data.Aero.Cnda/data.Aero.Clda*data.Aero.Clb/data.Aero.Cnb)))*r_bar;
        beta_trim = (data.Aero.Cnda*data.Aero.Clr-data.Aero.Cnr*data.Aero.Clda)/(data.Aero.Clda*(data.Aero.Cnb*(1-data.Aero.Cnda/data.Aero.Clda*data.Aero.Clb/data.Aero.Cnb)))*r_bar;

        d = 1e5;
        while d > eps
            % recalculate centripetal force, r, aileron trim and sideslip
            r = dpsi*cos(phi);
            r_bar = r*data.Geo.b/(2*V);
            delta_a_trim = (data.Aero.Clb*data.Aero.Cnr-data.Aero.Cnb*data.Aero.Clr)/(data.Aero.Clda*(data.Aero.Cnb*(1-data.Aero.Cnda/data.Aero.Clda*data.Aero.Clb/data.Aero.Cnb)))*r_bar;
            beta_trim = (data.Aero.Cnda*data.Aero.Clr-data.Aero.Cnr*data.Aero.Clda)/(data.Aero.Clda*(data.Aero.Cnb*(1-data.Aero.Cnda/data.Aero.Clda*data.Aero.Clb/data.Aero.Cnb)))*r_bar;
            
            % get a better estimate for bank angle (rearrange lecture 7A
            % slide 19)
            C_Y = data.Aero.Cyb*beta_trim + data.Aero.Cyr*r_bar;
            F_c =  data.Inertial.m*V/data.Geo.S/Q*r;
            C_L=data.Inertial.m*data.Inertial.g/(Q*data.Geo.S*cos(phi));
            
            % I think this should be divided by C_L but I can't get it to
            % converge
            % Results for now are reasonable i guess
            phi_next = (F_c-C_Y);
            
            d = abs(phi-phi_next);
            phi = abs(phi_next);
        end
            
        disp("Aileron deflection: " + rad2deg(delta_a_trim) + "deg")
        disp("Sideslip angle: " + rad2deg(beta_trim) + "deg")
        disp("Bank angle: " + rad2deg(phi) + "deg")
        disp("Stable: " + (delta_a_trim<0))
        
        disp("-----------------------------------")
    end
end