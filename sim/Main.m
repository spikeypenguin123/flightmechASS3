clear
close all
clc

addpath('Aircraft');
addpath('AircraftData');
addpath('Visualiser');

%% Configure

CONFIG = {};
CONFIG.debug = false; % bool
CONFIG.flight_plan = 1; % 1->8
CONFIG.CG = "CG1"; % CG1, CG2
CONFIG.V = 100; % 100, 180
CONFIG.visualise = true; % bool

CONFIG.t_start = 0; % don't change this
CONFIG.t_step = 0.1;
CONFIG.t_end = 10;
CONFIG.t = CONFIG.t_start:CONFIG.t_step:CONFIG.t_end;

%% Inititalise

aircraft = Initialisation(CONFIG.CG, CONFIG.V);

if CONFIG.visualise
    visualiser = initialise_visualiser(aircraft.state.x_e, aircraft.state.y_e,...
        aircraft.state.z_e, true, CONFIG.V); 
end

%% main loop

% TODO: trim the aircraft
% aircraft = Trim(aircraft);

for t = CONFIG.t
    [aircraft.controls.delta_T, aircraft.controls.delta_e, ...
        aircraft.controls.delta_a, aircraft.controls.delta_r]...
            = Controls(CONFIG.flight_plan, t);

    % get required data (gravity, wind, flow etc)
    
    % [rho, Q] = FlowProperties(altitude, ...
    %   sqrt(aircraft.u^2+aircraft.v^2+aircraft.w^2), aircraft.inertial.g);
    
        
    % alter the aircraft state here (do fancy calculations and integrations):
    
    
    
        
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
        disp(t)
        disp(aircraft.state)
    end
    
    % save the aircraft state as a point in time, for plots and analysis
    aircraft = save_vectors(aircraft);
end

if CONFIG.visualise
    visualiser.anim.close()
end

%% plots and analysis

% PlotData(aircraft.vectors);