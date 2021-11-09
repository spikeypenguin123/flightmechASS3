clc
clear
clear all

% load helper functions from flight sim
addpath('../sim')

mat1 = load('Longitudinal_Matrices_PC9_nominalCG1_100Kn_1000ft.mat');
mat2 = load('Longitudinal_Matrices_PC9_nominalCG1_180Kn_1000ft.mat');
mat3 = load('Longitudinal_Matrices_PC9_CG2_100Kn_1000ft.mat');

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
        
        if cg == "CG1" && speed == 100
            A_lona = mat1;
        elseif cg == "CG1" && speed == 180
            A_lona = mat2;
        else
            A_lona = mat3;
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
        % 8B+
        
        u1 = V; % delta_T
        theta1 = 1;    % DOUBLE CHECK
        A1 = Ixz/Ixx;
        B1 = Ixz/Izz;
        
        % Lateral State Constants
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
        NTb = 0; % Assumed zero for simplicity according to lecture 8B
        
        % Longitudinal State Matrix 
        % alpha form
        e_lona = eig(A_lona.A_Lon);
        [omega_lata, zeta_lata] = damp(e_lona);
        disp([newline, 'Alpha form matrix: ']);
        disp('Eigenvalues of longitudinal motion: ');
        disp(num2str(e_lona));
        disp('Natural Frequency: ');
        disp(num2str(omega_lata));
        disp('Damping Ratio: ');
        disp(num2str(zeta_lata));
        
        
        % Lateral State Matrices
        % Beta form
        A_latb = [Yb/u1 Yp/u1 (Yr/u1 - 1) (g*cos(theta1))/u1 0 ;
                 (Lb + A1*(Nb + NTb))/(1 - A1*B1) (Lp + A1*Np)/(1 - A1*B1) (Lr + A1*Nr)/(1 - A1*B1) 0 0 ;
                 (Nb + NTb + B1*Lb)/(1 - A1*B1) (Np + B1*Lp)/(1 - A1*B1) (Nr + B1*Lr)/(1 - A1*B1) 0 0 ;
                 0 1 tan(theta1) 0 0 ;
                 0 0 sec(theta1) 0 0 ];
             
        e_latb = eig(A_latb);
        [omega_latb, zeta_latb] = damp(e_latb); % Need to double check
        disp([newline, 'Beta form matrix: ']);
        disp('Eigenvalues of lateral motion: ');
        disp(num2str(e_latb));
        disp('Natural Frequency: ');
        disp(num2str(omega_latb));
        disp('Damping Ratio: ');
        disp(num2str(zeta_latb));
        

        
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


