clear
close all
clc

addpath('Aircraft');
addpath('AircraftData');
addpath('Controls');

set(groot,'defaulttextinterpreter','latex')
set(groot, 'defaultAxesTickLabelInterpreter','latex'); 
set(groot, 'defaultLegendInterpreter','latex');


%% Configure
j = 1;
a_100_1 = struct();
a_180_1 = struct();
a_100_2 = struct();
a_180_2 = struct();
for cg = ["CG1","CG2"]
for v = [100,180]   
CONFIG = {};
CONFIG.debug = false; % bool
CONFIG.flight_plan = 3; % 1->8
CONFIG.CG = cg; % CG1, CG2
CONFIG.V = v; % 100, 180
CONFIG.visualise = false; % bool
CONFIG.plot = false; % bool

CONFIG.t_start = 0; % don't change this
CONFIG.t_step = 0.1;
CONFIG.t_end = 200;
CONFIG.t = CONFIG.t_start:CONFIG.t_step:CONFIG.t_end;

%% Inititalise

aircraft = Initialisation(CONFIG.CG, CONFIG.V, CONFIG.debug);

if CONFIG.visualise
    visualiser = initialise_visualiser(aircraft.state.x_e, aircraft.state.y_e,...
        aircraft.state.z_e, true, CONFIG.V/3); 
end

%% main loop

% TODO: trim the aircraft
aircraft = Trim(aircraft);

dx_prev = zeros(13);

% remove the below line once the Trim function is complete.
% aircraft.controls = aircraft.trim;
aircraft.trim = aircraft.controls;

% dont remove this
[~, control_vec, ~] = get_vectors(aircraft);
aircraft.controls.vector = control_vec;

i = 1;
for t = CONFIG.t
    [aircraft.controls.delta_T, aircraft.controls.delta_e, ...
        aircraft.controls.delta_a, aircraft.controls.delta_r]...
            = Controls(CONFIG.flight_plan, t, i, aircraft.trim.delta_T,...
            aircraft.trim.delta_e, aircraft.trim.delta_a, aircraft.trim.delta_r);
        
    %% alter the aircraft state here (do fancy calculations and integrations):
    
    [X, dx_prev] = Integrate(aircraft, CONFIG.t_step, dx_prev, CONFIG.debug);
    
    aircraft.state.u        = X(1);     % (m/s)       
    aircraft.state.v        = X(2);     % (m/s)       
    aircraft.state.w        = X(3);     % (m/s)       
    aircraft.state.p        = X(4);     % (rad/s)     
    aircraft.state.q        = X(5);     % (rad/s)     
    aircraft.state.r        = X(6);     % (rad/s)     
    aircraft.state.quat(1)  = X(7);     % quat w     
    aircraft.state.quat(2)  = X(8);     % quat x      
    aircraft.state.quat(3)  = X(9);     % quat y      
    aircraft.state.quat(4)  = X(10);    % quat z 
    aircraft.state.x_e      = X(11);    % (m)         
    aircraft.state.y_e      = X(12);    % (m)         
    aircraft.state.z_e      = X(13);    % (m)    
    
    % make sure the euler angle representation is updated, since we have
    % updated the quaternions
    ea = q2e(aircraft.state.quat);
    aircraft.attitude.phi = ea(1);
    aircraft.attitude.theta = ea(2);
    aircraft.attitude.psi = ea(3);
        
    if CONFIG.visualise
        visualiser = add_frame(visualiser, aircraft.state.x_e, ...
            aircraft.state.y_e, aircraft.state.z_e, aircraft.attitude.phi, ...
            aircraft.attitude.theta, aircraft.attitude.psi, t);
    end
    
    if CONFIG.debug
        % log some stuff
        t
        aircraft.state
        aircraft.controls

        return
    end
    
    % save the aircraft state as a point in time, for plots and analysis
    if t~=0
        aircraft = save_vectors(aircraft);
    end

    
    i=i+1;

end
if j == 1
    a_100_1 = aircraft.vectors;
elseif j == 2
    a_180_1 = aircraft.vectors;
elseif j == 3
    a_100_2 = aircraft.vectors;
else
    a_180_2 = aircraft.vectors;
end
j=j+1;
end
end

if CONFIG.visualise
    visualiser.anim.close()
end

%% plots and analysis

if CONFIG.plot
    PlotData(aircraft.vectors, CONFIG.t);
end

t = CONFIG.t;

leg = ["CG1, 100KEAS", "CG1, 180KEAS", "CG2, 100KEAS", "CG2, 180KEAS"];

figure
set(gca, 'fontsize', 30)
hold on
plot(t,a_100_1.state(11,:),'LineWidth',2);
plot(t,a_180_1.state(11,:),'LineWidth',2);
plot(t,a_100_2.state(11,:),'LineWidth',2);
plot(t,a_180_2.state(11,:),'LineWidth',2);
grid minor
% title('Displacement ($x_{e}$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Position, (m)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
plot(t,a_100_1.state(12,:),'LineWidth',2);
plot(t,a_180_1.state(12,:),'LineWidth',2);
plot(t,a_100_2.state(12,:),'LineWidth',2);
plot(t,a_180_2.state(12,:),'LineWidth',2);
grid minor
% title('Displacement ($y_{e}$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Position, (m)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
plot(t,-a_100_1.state(13,:),'LineWidth',2);
plot(t,-a_180_1.state(13,:),'LineWidth',2);
plot(t,-a_100_2.state(13,:),'LineWidth',2);
plot(t,-a_180_2.state(13,:),'LineWidth',2);
grid minor
% title('Altitude ($-z_{e}$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Position, (m)','Interpreter','Latex');
legend(leg);

figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.state(4,:)),'LineWidth',2);
plot(t,rad2deg(a_180_1.state(4,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.state(4,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.state(4,:)),'LineWidth',2);
grid minor
% title('Angular Velocity about $x$ ($p$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angular Velocity ($^o/s$)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.state(5,:)),'LineWidth',2);
plot(t,rad2deg(a_180_1.state(5,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.state(5,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.state(5,:)),'LineWidth',2);
grid minor
% title('Angular Velocity about $y$ ($q$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angular Velocity ($^o/s$)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.state(6,:)),'LineWidth',2);
plot(t,rad2deg(a_180_1.state(6,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.state(6,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.state(6,:)),'LineWidth',2);
grid minor
% title('Angular Velocity about $z$ ($r$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angular Velocity ($^o/s$)','Interpreter','Latex');
legend(leg);


figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.attitude(2,:)),'LineWidth',2);
plot(t,rad2deg(a_180_1.attitude(2,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.attitude(2,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.attitude(2,:)),'LineWidth',2);
grid minor
% title('Pitch ($\theta$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angle, ($^o$)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.attitude(1,:)),'LineWidth',2);
plot(t,rad2deg(a_180_1.attitude(1,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.attitude(1,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.attitude(1,:)),'LineWidth',2);
grid minor
% title('Roll ($\phi$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angle, ($^o$)','Interpreter','Latex');
legend(leg);
figure
set(gca, 'fontsize', 30)
hold on
x = a_100_1.attitude(3,:);
x(x>0.1)=x(x>0.1)-2*pi;
plot(t,rad2deg(x),'LineWidth',2);
plot(t,rad2deg(a_180_1.attitude(3,:)),'LineWidth',2);
plot(t,rad2deg(a_100_2.attitude(3,:)),'LineWidth',2);
plot(t,rad2deg(a_180_2.attitude(3,:)),'LineWidth',2);
grid minor
% title('Yaw ($\psi$)');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Angle, ($^o$)','Interpreter','Latex');
legend(leg);


figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.control(3,:)),'k-','LineWidth',2);
grid minor
% title('Aileron Deflection');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Deflection ($^o$)','Interpreter','Latex');
figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.control(2,:)),'k-','LineWidth',2);
grid minor
% title('Elevator Deflection');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Deflection ($^o$)','Interpreter','Latex');
figure
set(gca, 'fontsize', 30)
hold on
plot(t,rad2deg(a_100_1.control(4,:)),'k-','LineWidth',2);
grid minor
% title('Rudder Deflection');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('Time, (s)','Interpreter','Latex');
ylabel('Deflection ($^o$)','Interpreter','Latex');


figure
set(gca, 'fontsize', 30)
hold on
plot(a_100_1.state(11,:),a_100_1.state(12,:),'LineWidth',2);
plot(a_180_1.state(11,:),a_180_1.state(12,:),'LineWidth',2);
plot(a_100_2.state(11,:),a_100_2.state(12,:),'LineWidth',2);
plot(a_180_2.state(11,:),a_180_2.state(12,:),'LineWidth',2);
grid minor
% title('Top-Down Trajectory');
han.XLabel.Visible='on';
han.YLabel.Visible='on';
xlabel('x, (m)','Interpreter','Latex');
ylabel('y (m)','Interpreter','Latex');
legend(leg);


