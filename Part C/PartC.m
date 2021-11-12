clc
clear
clear all

%% Default plot parameters

alw           = 1;                      % AxesLineWidth
fsz           = 20;                     % Fontsize
lw            = 1.4;                    % LineWidth
msz           = 5;                      % MarkerSize
leg_s         = 18;                     % Legend size
set(0,'defaultLineLineWidth',lw);       % set the default line width to lw
set(0,'defaultLineMarkerSize',msz);     % set the default line marker size to msz
set(0,'defaultLineLineWidth',lw);       % set the default line width to lw
set(0,'defaultLineMarkerSize',msz);     % set the default line marker size to msz

pos = [500 200 800 550];

%% Main Code

% TO DO: % Double check how to calculate time constant
         % Double check if zetas and omegas need to be absolute for
         % handling conditions

% load helper functions from flight sim
addpath('../sim')

mat1 = load('Longitudinal_Matrices_PC9_nominalCG1_100Kn_1000ft.mat');
mat2 = load('Longitudinal_Matrices_PC9_nominalCG1_180Kn_1000ft.mat');
mat3 = load('Longitudinal_Matrices_PC9_CG2_100Kn_1000ft.mat');
X1 = load('ICs_PC9_nominalCG1_100Kn_1000ft.mat');
X2 = load('ICs_PC9_nominalCG1_180Kn_1000ft.mat');
X3 = load('ICs_PC9_CG2_100Kn_1000ft.mat');

plotting = false; % Boolean for whether you want to plot or not

output.data_lon = [];
output.data_lat = [];
mat2lat = [];

% Go through each case
for cg = ["CG1", "CG2"]
    for speed = [100, 180]
        if cg == "CG2" && speed == 180
            % we only do 3 cases so lets call it here
            format short
                output.data_CG1_100_lon = [output.data_lon(:,1)];
                output.omega_zeta_CG1_100_lon = [real(output.data_lon(:,2:3))];
                output.data_CG1_180_lon = [output.data_lon(:,4)];
                output.omega_zeta_CG1_180_lon = [real(output.data_lon(:,5:6))];
                output.data_CG2_100_lon = [output.data_lon(:,7)];
                output.omega_zeta_CG2_100_lon = [real(output.data_lon(:,8:9))];

                output.data_CG1_100_lat = [output.data_lat(:,1)];
                output.omega_zeta_CG1_100_lat = [real(output.data_lat(:,2:3))];
                output.data_CG1_180_lat = [output.data_lat(:,4)];
                output.omega_zeta_CG1_180_lat = [real(output.data_lat(:,5:6))];
                output.data_CG2_100_lat = [output.data_lat(:,7)];
                output.omega_zeta_CG2_100_lat = [real(output.data_lat(:,8:9))];
                
                output.omega_zeta = [real(output.data_lon(:,2:3)) real(output.data_lon(:,5:6)) real(output.data_lon(:,8:9)) ...
                                    real(output.data_lat(:,2:3)) real(output.data_lat(:,5:6)) real(output.data_lat(:,8:9))];

                output.check = output.data_lon;
            return
        end
        disp("Case: " + cg + "@" + speed + "kts")

        if cg == "CG1"
            data = aero3560_LoadFlightDataPC9_nominalCG1();
        else
            data = aero3560_LoadFlightDataPC9_CG2();
        end
        
        if cg == "CG1" && speed == 100
            lon = mat1;
            att_eul = q2e(X1.X0(7:10)/norm(X1.X0(7:10)));
            theta = att_eul(2);
        elseif cg == "CG1" && speed == 180
            lon = mat2;
            att_eul = q2e(X2.X0(7:10)/norm(X2.X0(7:10)));
            theta = att_eul(2);
        else
            lon = mat3;
            att_eul = q2e(X3.X0(7:10)/norm(X3.X0(7:10)));
            theta = att_eul(2);
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
        disp([newline,'Part A']);
        % Using Dynamic Stability Matrix from Week 10A and terms from Week
        % 8B
        
        u1 = V; % delta_T
        theta1 = theta;    % DOUBLE CHECK
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
        Ndr = (Q*S*b*Cndr)/Izz;
        Ldr = (Q*S*b*Cldr)/Ixx;
        Lb = (Q*S*b*Clb)/Ixx;
        Lp = (Q*S*b^2*Clp)/(2*Ixx*u1);
        Lr = (Q*S*b^2*Clr)/(2*Ixx*u1);
        NTb = 0; % Assumed zero for simplicity according to lecture 8B
        
        % Longitudinal State Matrix 
        A_lon = lon.A_Lon;
        B_lon = lon.B_Lon;
        % w form
        e_lon = eig(lon.A_Lon);
        [omega_lon, zeta_lon] = damp(e_lon);
        disp('w form matrix: ');
        disp('Eigenvalues of longitudinal motion: ');
        disp(num2str(e_lon));
        disp('Natural Frequency: ');
        disp(num2str(omega_lon));
        disp('Damping Ratio: ');
        disp(num2str(zeta_lon));
        
        
        % Lateral State Matrices
        % Beta form
        A_lat = [Yb/u1 Yp/u1 (Yr/u1 - 1) (g*cos(theta1))/u1 0 ;
                 (Lb + A1*(Nb + NTb))/(1 - A1*B1) (Lp + A1*Np)/(1 - A1*B1) (Lr + A1*Nr)/(1 - A1*B1) 0 0 ;
                 (Nb + NTb + B1*Lb)/(1 - A1*B1) (Np + B1*Lp)/(1 - A1*B1) (Nr + B1*Lr)/(1 - A1*B1) 0 0 ;
                 0 1 tan(theta1) 0 0 ;
                 0 0 sec(theta1) 0 0 ];
        B_lat = [Yda/u1 Ydr/u1 ; 
                  ((Lda + A1*Nda)/(1 - A1*B1)) ((Ldr + A1*Ndr)/(1 - A1*B1)) ;
                  ((Nda + B1*Lda)/(1 - A1*B1)) ((Ndr + B1*Ldr)/(1 - A1*B1)) ; 
                  0 0 ;
                  0 0];
             
        e_lat = eig(A_lat);
        [omega_lat, zeta_lat] = damp(e_lat); % Need to double check
        disp([newline, 'Beta form matrix: ']);
        disp('Eigenvalues of lateral motion: ');
        disp(num2str(e_lat));
        disp('Natural Frequency: ');
        disp(num2str(omega_lat));
        disp('Damping Ratio: ');
        disp(num2str(zeta_lat));
        
        
        %% Part B   -- Check for positive values
        disp([newline,'Part B']);       
        
        % Calculating time constants for Part F
        tau_lon = 1./omega_lon;
        tau_lat = 1./omega_lat;

        disp('Longitudinal Modes of motion');
        % Checking if it is a short period mode, phugoid mode
        % Index position
        % idx 1 = u
        % idx 2 = w
        % idx 3 = q
        % idx 4 = theta 
        % idx 5 = ze
        for i = 1:length(omega_lon)
            if isreal(e_lon(i)) == 0
                if omega_lon(i) >= 1 && omega_lon(i) <= 6 && zeta_lon(i) >= 0.5 && zeta_lon(i) <= 0.8
                    disp(['State: ', num2str(i), ' |', ' SPM', ' |', ' Eig: ' ...
                        num2str(e_lon(i)), ' |',' Omega: ', num2str(omega_lon(i)), ' |', ' Zeta: ', num2str(zeta_lon(i))]);
                    w_SPM = omega_lon(i);
                    z_SPM = zeta_lon(i);
                    tau_SPM = tau_lon(i);
                elseif omega_lon(i) >= 0.1 && omega_lon(i) <= 0.6
                    disp(['State: ', num2str(i), ' |', ' Phugoid', ' |', ' Eig: ' ...
                        num2str(e_lon(i)),' |', ' Omega: ', num2str(omega_lon(i)), ' |', ' Zeta: ', num2str(zeta_lon(i))]);
                    w_Phu = omega_lon(i);
                    z_Phu = zeta_lon(i);
                    tau_Phu = tau_lon(i);
                end
            else
                disp(['State: ', num2str(i), ' |', ' Altitude con/div', ' |', ' Eig: ' ...
                        num2str(e_lon(i)),' |', ' Omega: ', num2str(omega_lon(i)), ' |', ' Zeta: ', num2str(zeta_lon(i))]);
            end
        end 
                    
        disp([newline, 'Lateral Modes of Motion']);
        % Checking if it is a roll mode, dutch roll mode or spiral mode
        % Index position
        % idx 1 = beta
        % idx 2 = p
        % idx 3 = r
        % idx 4 = phi 
        % idx 5 = psi
        for i = 1:length(omega_lat)
            if isreal(e_lat(i)) == 0
                disp(['State: ', num2str(i), ' |',' Dutch Roll',  ' |', ' Eig: ' ...
                        num2str(e_lat(i)),' |', ' Omega: ', num2str(omega_lat(i)), ' |', ' Zeta: ', num2str(zeta_lat(i))]);
                z_DR = zeta_lat(i);
                w_DR = omega_lat(i);
                tau_DR = tau_lat(i);
            elseif zeta_lat(i) == -1
                disp(['State: ', num2str(i), ' |',' Spiral Mode',  ' |', ' Eig: ' ...
                        num2str(e_lat(i)),' |', ' Omega: ', num2str(omega_lat(i)), ' |', ' Zeta: ', num2str(zeta_lat(i))]);
                z_SM = zeta_lat(i);
                w_SM = omega_lat(i);
                tau_SM = tau_lat(i);
            elseif zeta_lat(i) == 1
                disp(['State: ', num2str(i), ' |',' Roll Mode',  ' |', ' Eig: ' ...
                        num2str(e_lat(i)),' |', ' Omega: ', num2str(omega_lat(i)), ' |', ' Zeta: ', num2str(zeta_lat(i))]);
                z_RM = zeta_lat(i);
                w_RM = omega_lat(i);
                tau_RM = tau_lat(i);
            end
        end
        
        %% Part C
%         disp([newline,'Part C']);
        
        % Euler integration
        % Control deflection of 5 deg held for 0.5s and then returned to 0
        % deg
            
        % Conditions for Euler Integration
        T = 80;
        time_def = 5; % Time at which 0.5 second deflection occurs
        dT = 0.1;
        Steps = T/dT;
        
        % States
        % Longitudinal - [u w q theta z]
        % Lateral - [beta p r phi psi]
        X_lon = [0 0 0 0 0]'; % Assigning intial values for states
        X_lat = [0 0 0 0 0]';
        
        % Control inputs
        % Longitudinal Case - Throttle and Elevator
        % Lateral Directional Case - Aileron and Rudder
        def = [5 5]'*pi/180;
        def_zero = [0 0]'*pi/180;
        
        % Longitudinal Case and Lateral-Directional Case
        for i = 1:Steps
            
            time = i*dT;
            
            if time >= time_def && time <= time_def + 0.5
                Xdot_lon = A_lon*X_lon(:,i) + B_lon*def;
                Xdot_lat = A_lat*X_lat(:,i) + B_lat*def;
            else
                Xdot_lon = A_lon*X_lon(:,i) + B_lon*def_zero;  
                Xdot_lat = A_lat*X_lat(:,i) + B_lat*def_zero;
            end
            X_lon(:,i + 1) = X_lon(:,i) + dT*Xdot_lon;
            X_lat(:,i + 1) = X_lat(:,i) + dT*Xdot_lat;
        end
            
                
        %% Part D
%         disp([newline,'Part D']);
        
        % Time history and comparison with eigenvalues
        Time = linspace(0,T,length(X_lon));
        
        if plotting == 1
            
            % Longitudinal Case
            figure(1)
            plot(Time,X_lon(1,:)) % u
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('$\frac{m}{s}$','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            saveas(gcf,['Longitudinal_vel_' + cg + '@' + num2str(speed) + 'kts' + '_dT' + num2str(rad2deg(def(1))) + 'de' + num2str(rad2deg(def(2))) + '.png']);
            hold off
            
            figure(2)
            plot(Time,rad2deg(X_lon(3,:))) % q
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('$\frac{deg}{s}$','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            saveas(gcf,['Longitudinal_rates_' + cg + '@' + num2str(speed) + 'kts' + '_dT' + num2str(rad2deg(def(1))) + 'de' + num2str(rad2deg(def(2))) + '.png']);
            hold off
            
            figure(3)
            plot(Time,rad2deg(X_lon(2,:))./u1) % w form but outputting alpha (using alpha = w/u1)
            hold on
            plot(Time,rad2deg(X_lon(4,:))) % theta
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('deg','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            leg = legend('$\alpha$','$\theta$',...
                'Interpreter','latex','Orientation','Horizontal','Location','Best');
            set(leg,'FontSize',leg_s);
            saveas(gcf,['Longitudinal_angles_' + cg + '@' + num2str(speed) + 'kts' + '_dT' + num2str(rad2deg(def(1))) + 'de' + num2str(rad2deg(def(2))) + '.png']);
            hold off
            
            figure(4)
            plot(Time,X_lon(5,:)) % ze
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('m','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            saveas(gcf,['Longitudinal_pos_' + cg + '@' + num2str(speed) + 'kts' + '_dT' + num2str(rad2deg(def(1))) + 'de' + num2str(rad2deg(def(2))) + '.png']);
            hold off

            %-------------------------------------------------------%
            
            % Lateral Case
            figure(5)
            plot(Time,rad2deg(X_lat(1,:))) % beta
            hold on
            plot(Time,rad2deg(X_lat(4,:))) % phi
            plot(Time,rad2deg(X_lat(5,:))) % psi
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('deg','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            leg = legend('$\beta$','$\phi$','$\psi$',...
                'Interpreter','latex','Orientation','Horizontal','Location','Best');
            set(leg,'FontSize',leg_s);
            saveas(gcf,['Lateral_angles_' + cg + '@' + num2str(speed) + 'kts' + '_da' + num2str(rad2deg(def(1))) + 'dr' + num2str(rad2deg(def(2))) + '.png']);
            hold off
            
            figure(6)         
            plot(Time,rad2deg(X_lat(2,:))) % p 
            hold on
            plot(Time,rad2deg(X_lat(3,:))) % r
            grid on; grid minor; box on
            xlabel('Time (s)','Interpreter','latex')
            ylabel('$\frac{deg}{s}$','Interpreter','latex')
            set(gca,'FontSize',fsz)
            set(gcf,'Position',pos)
            leg = legend('p','r',...
                'Interpreter','latex','Orientation','Horizontal','Location','Best');
            set(leg,'FontSize',leg_s);
            saveas(gcf,['Lateral_rates_' + cg + '@' + num2str(speed) + 'kts' + '_da' + num2str(rad2deg(def(1))) + 'dr' + num2str(rad2deg(def(2))) + '.png']);
            hold off
            
        end
        
        
        %% Part E
%         disp([newline,'Part E']);
        % Checking effect of control surfaces on the modes of motion
        % Probably have to vary the control inputs, just use part d
        % Explanation
        
        

        
        %% Part F
        disp([newline,'Part F']);
        % Handling qualities of the aircraft
        % Using Military Specifications from Week 12A
        % MIL-F-8785C
        
        % PC9 - Class 1 Aircraft
        % Need to check with flight condition the PC9 is in
        % At the moment checking all of them
        
        
        % Longitudinal - Modes: SPM, Phugoid
        disp('Longitudinal Case');
        % Category A
        disp('Category A');
        
        % SPM
        if z_SPM >= 0.35 && z_SPM <= 1.3
            disp('SPM Handling: Level 1');
        elseif z_SPM >= 0.25 && z_SPM <= 2
            disp('SPM Handling: Level 2');
        elseif z_SPM >= 0.15
            disp('SPM Handling: Level 3');
        else
            disp('SPM No Handling Level');
        end
            
        % Phugoid
        if z_Phu >= 0.04
            disp('Phugoid Handling: Level 1');
        elseif z_Phu >= 0
            disp('Phugoid Handling: Level 2');
        else
            disp('Phugoid Level 3 or no handling level');
        end
        
        % Category B
        disp([newline, 'Category B']);
        
        % SPM
        if z_SPM >= 0.3 && z_SPM <= 2
            disp('SPM Handling: Level 1');
        elseif z_SPM >= 0.2 && z_SPM <= 2
            disp('SPM Handling: Level 2');
        elseif z_SPM >= 0.15
            disp('SPM Handling: Level 3');
        else
            disp('SPM No Handling Level');
        end
        
        % Phugoid
        if z_Phu >= 0.04
            disp('Phugoid Handling: Level 1');
        elseif z_Phu >= 0
            disp('Phugoid Handling: Level 2');
        else
            disp('Phugoid Level 3 or no handling level');
        end
        
        
        % Category C
        disp([newline, 'Category C']);
        
        % SPM
        if z_SPM >= 0.35 && z_SPM <= 1.3
            disp('SPM Handling: Level 1');
        elseif z_SPM >= 0.25 && z_SPM <= 2
            disp('SPM Handling: Level 2');
        elseif z_SPM >= 0.15
            disp('SPM Handling: Level 3');
        else
            disp('SPM No Handling Level');
        end
        
        % Phugoid
        if z_Phu >= 0.04
            disp('Phugoid Handling: Level 1');
        elseif z_Phu >= 0
            disp('Phugoid Handling: Level 2');
        else
            disp('Phugoid Level 3 or no handling level');
        end
        
        
        %-------------------------------------------------------------%
        
        % Lateral - Modes: Dutch Mode, Roll Mode, Spiral Mode
        disp([newline, 'Lateral Case']);
        
        % Calculating time for bank angle disturbance to double for spiral
        % mode condition - DOUBLE CHECK (using graph)
        time_start = []; time_end = [];
        for i = 1:length(X_lat(4,:))
            if abs(X_lat(4,i)) >= (20*pi)/180
                time_start = [time_start Time(i)];
            end
            if abs(X_lat(4,i)) >= (40*pi)/180
                time_end = [time_end Time(i)];
            end
        end  
        time_doubphi = time_end(1) - time_start(1);
        
        % Category A
        disp('Category A');
        
        % Dutch Roll
        if z_DR >= 0.19 && w_DR >= 1
            disp('Duth Roll Handling: Level 1');
        elseif z_DR >= 0.02 && z_DR*w_DR >= 0.05 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 2');
        elseif z_DR >= 0 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 3');
        end
        
        % Roll Mode - time constant
        if tau_RM <= 1
            disp('Roll Mode Handling: Level 1');
        elseif tau_RM <= 1.4
            disp('Roll Mode Handling: Level 2');
        elseif tau_RM <= 10
            disp('Roll Mode Handling: Level 3');
        end
        
        % Spiral Mode - Bank angle disturbance 
        if time_doubphi > 8 && time_doubphi <= 12
            disp('Spiral Mode Handling: Level 1');
        elseif time_doubphi > 4 && time_doubphi <= 8
            disp('Spiral Mode Handling: Level 2');
        elseif time_doubphi > 0 && time_doubphi <= 4
            disp('Spiral Mode Handling: Level 3');
        end

        
        % Category B
        disp([newline, 'Category B']);
        
        % Dutch Roll
        if z_DR >= 0.08 && z_DR*w_DR >= 0.15 && w_DR >= 4
            disp('Duth Roll Handling: Level 1');
        elseif z_DR >= 0.02 && z_DR*w_DR >= 0.05 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 2');
        elseif z_DR >= 0 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 3');
        end
        
        % Roll Mode - time constant
        if tau_RM <= 1.4
            disp('Roll Mode Handling: Level 1');
        elseif tau_RM <= 3
            disp('Roll Mode Handling: Level 2');
        elseif tau_RM <= 10
            disp('Roll Mode Handling: Level 3');
        end
        
        % Spiral Mode - Bank angle disturbance
        if time_doubphi > 8 && time_doubphi <= 20
            disp('Spiral Mode Handling: Level 1');
        elseif time_doubphi > 4 && time_doubphi <= 8
            disp('Spiral Mode Handling: Level 2');
        elseif time_doubphi > 0 && time_doubphi <= 4
            disp('Spiral Mode Handling: Level 3');
        end
        
        
        % Category C
        disp([newline, 'Category C']);
        
        % Dutch Roll
        if z_DR >= 0.08 && z_DR*w_DR >= 0.05 && w_DR >= 1
            disp('Duth Roll Handling: Level 1');
        elseif z_DR >= 0.02 && z_DR*w_DR >= 0.05 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 2');
        elseif z_DR >= 0 && w_DR >= 0.4
            disp('Dutch Roll Handling: Level 3');
        end
        
        % Roll Mode - time constant
        if tau_RM <= 1
            disp('Roll Mode Handling: Level 1');
        elseif tau_RM <= 1.4
            disp('Roll Mode Handling: Level 2');
        elseif tau_RM <= 10
            disp('Roll Mode Handling: Level 3');
        end
        
        % Spiral Mode - Bank angle disturbance
        if time_doubphi > 8 && time_doubphi <= 12
            disp('Spiral Mode Handling: Level 1');
        elseif time_doubphi > 4 && time_doubphi <= 8
            disp('Spiral Mode Handling: Level 2');
        elseif time_doubphi > 0 && time_doubphi <= 4
            disp('Spiral Mode Handling: Level 3');
        end
        
        
        disp([newline, '------------------------------------']);
        
        % Compiling data
        output.data_lon = [output.data_lon e_lon omega_lon zeta_lon];
        output.data_lat = [output.data_lat e_lat omega_lat zeta_lat];
        
    end
end

