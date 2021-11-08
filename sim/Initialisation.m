function [aircraft] = Initialisation(cg,speed,debug)
    % Instantiate the aircraft struct based on parameters
    % cg (string): [CG1|CG2]
    % speed (int): [100|180] Knots, true air speed

    % load AircraftData;
    
    if cg == "CG1"
        cg = "nominalCG1";
    end
    
    load("ICs_PC9_"+cg+"_"+speed+"Kn_1000ft.mat");
    
    aircraft = {};
    
    if debug
        disp("Running in debug mode")
        % alter initial states to match debug variables
        X0=X0+0.1;
        U0=U0+0.1;
    end
        
    % aircraft state
    % currently the given data isn't how it's described in the PDF (12
    % values instead of 13)
    aircraft.state = {};
    aircraft.state.u = X0(1);
    aircraft.state.v = X0(2);
    aircraft.state.w = X0(3);
    aircraft.state.p = X0(4);
    aircraft.state.q = X0(5);
    aircraft.state.r = X0(6);
    aircraft.state.quat = e2q([X0(7);X0(8);X0(9)]);
    aircraft.state.x_e = X0(10);
    aircraft.state.y_e = X0(11);
    aircraft.state.z_e = X0(12);
    
    % aircraft control inputs
    aircraft.controls = {};
    aircraft.controls.delta_T = 0;
    aircraft.controls.delta_e = 0;
    aircraft.controls.delta_a = 0;
    aircraft.controls.delta_r = 0;
    
    % aircraft trim settings for initial conditions (used for checking)
    aircraft.trim = {};
    aircraft.trim.delta_T = U0(1);
    aircraft.trim.delta_e = U0(2);
    aircraft.trim.delta_a = U0(3);
    aircraft.trim.delta_r = U0(4);    
    
    % aircraft attitude
    aircraft.attitude = {};
    aircraft.attitude.phi = X0(7);
    aircraft.attitude.theta = X0(8);
    aircraft.attitude.psi = X0(9);
    
    % aircraft vectors (store data over time)
    [state_vec, control_vec, attitude_vec] = get_vectors(aircraft);
    aircraft.vectors.state = state_vec;
    aircraft.vectors.control = control_vec;
    aircraft.vectors.attitude = attitude_vec;
    
    % Load data and copy over to aircraft
    if cg == "nominalCG1"
        flight_data = aero3560_LoadFlightDataPC9_nominalCG1();
    else
        flight_data = aero3560_LoadFlightDataPC9_CG2();
    end
    
    aircraft.inertial = flight_data.Inertial;
    aircraft.geo = flight_data.Geo;
    aircraft.prop = flight_data.Prop;
    aircraft.control_limits = flight_data.ControlLimits;
    aircraft.aero = flight_data.Aero;
    
end

