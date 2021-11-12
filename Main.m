clear
close all
clc

addpath('Aircraft');
addpath('AircraftData');
addpath('Visualiser');
addpath('Controls');

%% Configure

% array of simulation times per flight plan
SIM_TIMES = [200 200 200 100 100 40 25 20];

CONFIG = {};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% CHANGE THESE %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
CONFIG.flight_plan = 5; % <--- change this to run different maneuvers. int,1-8
CONFIG.visualise = true; % <--- change this to turn on the visualiser. bool
CONFIG.plot = true; % bool
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


CONFIG.debug = false; % bool
CONFIG.CG = "CG1"; % CG1, CG2
CONFIG.V = 180; % 100, 180


CONFIG.t_start = 0; % don't change this
CONFIG.t_step = 0.1; % don't change this
CONFIG.t_end = SIM_TIMES(CONFIG.flight_plan);

CONFIG.t = CONFIG.t_start:CONFIG.t_step:CONFIG.t_end;

disp("Running test case " + CONFIG.flight_plan + " for " + CONFIG.CG + "@" + CONFIG.V + "kts");

%% Inititalise

aircraft = Initialisation(CONFIG.CG, CONFIG.V, CONFIG.debug);

if CONFIG.visualise
    visualiser = initialise_visualiser(aircraft.state.x_e, aircraft.state.y_e,...
    aircraft.state.z_e, true, CONFIG.V/10); 
end

%% main loop

% trim the aircraft
aircraft = Trim(aircraft);
aircraft.trim = aircraft.controls;

% initialise dx vector
dx_prev = zeros(13);

% save down initial state vector
[~, control_vec, ~] = get_vectors(aircraft);
aircraft.controls.vector = control_vec;

i = 1;
for t = CONFIG.t
    [aircraft.controls.delta_T, aircraft.controls.delta_e, ...
        aircraft.controls.delta_a, aircraft.controls.delta_r]...
            = Controls(CONFIG.flight_plan, t, i, aircraft.trim.delta_T,...
            aircraft.trim.delta_e, aircraft.trim.delta_a, aircraft.trim.delta_r);
        
    % calculate the next state vector X
    [X, dx_prev] = Integrate(aircraft, CONFIG.t_step, dx_prev, CONFIG.debug);
    
    % unpack the state
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
        disp(t);
        disp(aircraft.state);
        disp(aircraft.controls);

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
    % what you want to call the plots in input 3 and boolean to check
    % whether you want to save plots to file or not in input 4
    PlotData(aircraft.vectors, CONFIG.t, "maneuver"+CONFIG.flight_plan, false);
end



