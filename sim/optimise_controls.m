function dx = optimise_controls(controls)

    STATE = [91.5792078246906;-2.02668477641255;-2.28005080496474;-1.05684241158796;0.0497374287696083;0.130153326639134;0.909124529486464;0.413625185871887;-0.0483397898983037;0.00899874930796433;467.871658324774;-5.42332106879390;-308.358752511102];

    addpath('Aircraft');
    addpath('AircraftData');
    % addpath('Visualiser');
    % addpath('Control_GUI');

    % Run the control GUI
    % Control_GUI


    %% Configure

    CONFIG = {};
    CONFIG.debug = false; % bool
    CONFIG.flight_plan = 82; % 1->8
    CONFIG.CG = "CG2"; % CG1, CG2
    CONFIG.V = 180; % 100, 180
    CONFIG.visualise = false; % bool
    CONFIG.plot = false; % bool

    CONFIG.t_start = 0; % don't change this
    CONFIG.t_step = 0.1;
    CONFIG.t_end = 5;
    CONFIG.t = CONFIG.t_start:CONFIG.t_step:CONFIG.t_end;

    %% Inititalise

%     aircraft = Initialisation(CONFIG.CG, CONFIG.V, CONFIG.debug);
    aircraft = Initialisation_With_State(CONFIG.CG, CONFIG.V, CONFIG.debug, STATE);

    if CONFIG.visualise
        visualiser = initialise_visualiser(aircraft.state.x_e, aircraft.state.y_e,...
            aircraft.state.z_e, true, CONFIG.V/3); 
    end

    %% main loop

    % TODO: trim the aircraft
    % aircraft = Trim(aircraft);

    dx_prev = zeros(13);

    % remove the below line once the Trim function is complete.
    aircraft.controls = aircraft.trim;

    % dont remove this
    [~, control_vec, ~] = get_vectors(aircraft);
    aircraft.controls.vector = control_vec;

    i = 1;
    for t = CONFIG.t
        c = controls(:,i);
        if CONFIG.flight_plan == 81
            aircraft.controls.delta_T = bound(c(1) + aircraft.trim.delta_T, aircraft.control_limits.Lower(1),aircraft.control_limits.Upper(1));
            aircraft.controls.delta_e = bound(c(2) + aircraft.trim.delta_e, aircraft.control_limits.Lower(2),aircraft.control_limits.Upper(2));
            aircraft.controls.delta_a = deg2rad(7.5) + aircraft.trim.delta_a;
            aircraft.controls.delta_r = bound(c(4) + aircraft.trim.delta_r, aircraft.control_limits.Lower(4),aircraft.control_limits.Upper(4));
        end
        
        if CONFIG.flight_plan == 82
            aircraft.controls.delta_T = bound(c(1) + aircraft.trim.delta_T, aircraft.control_limits.Lower(1),aircraft.control_limits.Upper(1));
            aircraft.controls.delta_e = bound(c(2) + aircraft.trim.delta_e, aircraft.control_limits.Lower(2),aircraft.control_limits.Upper(2));
            aircraft.controls.delta_a = bound(c(3) + aircraft.trim.delta_a, aircraft.control_limits.Lower(3),aircraft.control_limits.Upper(3));
            aircraft.controls.delta_r = bound(c(4) + aircraft.trim.delta_r, aircraft.control_limits.Lower(4),aircraft.control_limits.Upper(4));
        end
        
        
        %% get required data (gravity, wind, flow etc)
    %     AngularRates = AngularRates(aircraft.vectors.state,Xd);
    %     [CL, CD, F_B, M_B, F_G, F_T, Pmax] = AllForces(aircraft,aircraft.vectors.state,aircraft.vectors.control,AngularRates); 
        % get velocity magnitude
    %     V = sqrt(aircraft.state.u^2+aircraft.state.v^2+aircraft.state.w^2);
    %     
    %     [rho, Q] = FlowProperties(aircraft,V);
    %   
    %     G_body = Gravity(aircraft.inertial.g, aircraft.state.quat, aircraft.inertial.m);
    %     
    %     [CL,CD] = WindForces(aircraft,X,U,alpha,V,AngularRates);
    %     
    %     [Pmax, T] = PropForces(rho, V, aircraft.controls.delta_T, aircraft.prop);

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
        
        i = i + 1;
    end

    if CONFIG.visualise
        visualiser.anim.close()
    end

    %% plots and analysis

    if CONFIG.plot
        PlotData(aircraft.vectors, CONFIG.t);
    end
    
    if CONFIG.flight_plan == 81
        % keep y, z at zero
        dx = sum(abs(aircraft.vectors.state(12,1)-aircraft.vectors.state(12,12:end)))+sum(abs(aircraft.vectors.state(13,1)-aircraft.vectors.state(13,13:end)));
    end
    if CONFIG.flight_plan == 82
        a = q2e(aircraft.vectors.state(7:10,:));
        % get phi, y, z to zero
        dx = sum(abs(a(1,:)))*10 ...
            +sum(abs(aircraft.vectors.state(12,:)))...
            +sum(abs(-305-aircraft.vectors.state(13,13:end)));
    end
    
end

