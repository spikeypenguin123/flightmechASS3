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
        
        g = 9.81;
        Q = 0.5*rho*V^2;
        
        Inertial = data.Inertial;
        CLim = data.ControlLimits;
        Aero = data.Aero;
        Geom = data.Geo;
        Prop = data.Prop;
        
        %% Load in Aircraft Data
        % Longitudinal
        CLa = Aero.CLa;
        Cma = Aero.Cma;
        Cmq = Aero.Cmq;
        Cmde = Aero.Cmde;
        
        % Lateral 
        Cyb = Aero.Cyb;
        Cyp = Aero.Cyp;
        Cyr = Aero.Cyr;
        Cyda = Aero.Cyda;
        Cnda = Aero.Cnda;
        Clda = Aero.Clda;
        Cnb = Aero.Cnb;
        Cnp = Aero.Cnp;
        Cnr = Aero.Cnr;
        Cydr = Aero.Cydr;
        Cndr = Aero.Cndr;
        Cldr = Aero.Cldr;
        Clb = Aero.Clb;
        Clp = Aero.Clp;
        Clr = Aero.Clr;
        
        % Geometry
        S = Geom.S;
        b = Geom.b;
        c = Geom.c;
        
        % Inertial
        m = Inertial.m;
        Izz = Inertial.Izz;
        Ixx = Inertial.Ixx;
        Ixz = Inertial.Ixz;
        
        
        %% Part A
        disp('Part A');
        % Using Dynamic Stability Matrix from Week 10A and terms from Week
        % 8B
        
        u1 = 1; % delta_T
        theta1 = 1;
        A1 = Ixz/Ixx;
        B1 = Ixz/Izz;
        
        % Longitudinal State Matrix
        
        
        % Lateral State Matrix
        Yb = (Q*S*Cyb)/m;
        Yp = (Q*S*b*Cyp)/(2*m*u1);
        Yr = (Q*S*b*Cyr)/(2*m*u1);
        Yda = (Q*S*Cyda)/m;
        Nda = (Q*S*b*Cnda)/Izz;
        Lda = (Q*S*b*Clda)/Ixx;
        Nb = (Q*S*b*Cnb)/Izz;
        Np = (Q*S*b^2*Cnp)/(2*Izz*u1);
        Nr = (Q*S*Cydr)/(2*Izz*u1);
        Ydr = (Q*S*b*Cndr)/Izz;
        Ldr = (Q*S*b*Cldr)/Ixx;
        Lb = (Q*S*b*Clb)/Ixx;
        Lp = (Q*S*b^2*Clp)/(2*Ixx*u1);
        Lr = (Q*S*b^2*Clr)/(2*Ixx*u1);
        NTb = 1; % Double check
        
        A_lat = [Yb/u1 Yp/u1 (Yr/u1 - 1) (g*cos(theta1))/u1 0 ;
                 (Lb + A1*(Nb + NTb))/(1 - A1*B1) (Lp + A1*Np)/(1 - A1*B1) (Lr + A1*Nr)/(1 - A1*B1) 0 0 ;
                 (Nb + NTb + B1*Lb)/(1 - A1*B1) (Np + B1*Lp)/(1 - A1*B1) (Nr + B1*Lr)/(1 - A1*B1) 0 0 ;
                 0 1 tan(theta1) 0 0 ;
                 0 0 sec(theta1) 0 0 ];
             
        e_lat = eig(A_lat);
        [omega_lat, zeta_lat] = damp(e_lat); % Need to double check
        disp('Eigenvalues of lateral motion: ');
        disp(num2str(e_lat));
        disp('Natural Frequency: ');
        disp(num2str(omega_lat));
        disp('Damping Ratio: ');
        disp(num2str(zeta_lat));

        
        %% Part B
        disp('Part B');
        
        
        
        %% Part C
        disp('Part C');
        

        
        %% Part D
        disp('Part D');
        

        
        %% Part E
        disp('Part E');
        

        
        %% Part F
        disp('Part F');
        
        
        
        disp('------------------------------------');
        
    end
end


