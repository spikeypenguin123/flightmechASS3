clear
close all
clc

addpath('Aircraft');
addpath('AircraftData');
addpath('Visualiser');
addpath('Control_GUI');
addpath('Controls');

% Run the control GUI
% Control_GUI
% Use the provided trim U0 in the GUI and save this
% Will be loaded everytime you want to run that particular sim

% the frequency for the filtered U0 can be a discussion point in the report

% U_linear for the first 3
% U_smoothed for the maneurvers 4-8 (dutch roll and spiral mode are exciting)


%% Part C Task 1c is using the U_linear
    % could also use the U_filter and compare for discussion

%% Configure

CONFIG = {};
CONFIG.debug = false; % bool
CONFIG.flight_plan = 1; % 1->8
CONFIG.CG = "CG1"; % CG1, CG2
CONFIG.V = 100; % 100, 180
CONFIG.visualise = true; % bool
CONFIG.plot = true; % bool

CONFIG.t_start = 0; % don't change this
CONFIG.t_step = 0.1;
CONFIG.t_end = 20;
CONFIG.t = CONFIG.t_start:CONFIG.t_step:CONFIG.t_end;

%% Inititalise

aircraft = Initialisation(CONFIG.CG, CONFIG.V, CONFIG.debug);

if CONFIG.visualise
    visualiser = initialise_visualiser(aircraft.state.x_e, aircraft.state.y_e,...
        aircraft.state.z_e, true, CONFIG.V/3); 
end

%% main loop

% TODO: trim the aircraft
aircraft = TrimTest(aircraft);

aircraft.controls
aircraft.trim
return

dx_prev = zeros(13);

% remove the below line once the Trim function is complete.
% aircraft.controls = aircraft.trim;

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

if CONFIG.visualise
    visualiser.anim.close()
end

%% plots and analysis

if CONFIG.plot
    PlotData(aircraft.vectors, CONFIG.t);
end



