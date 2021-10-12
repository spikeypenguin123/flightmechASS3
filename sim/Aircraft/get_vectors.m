function [state_vec, control_vec, attitude_vec] = get_vectors(aircraft)
    % Vectorises the aircraft variables

    state_vec = [
        aircraft.state.u;
        aircraft.state.v;
        aircraft.state.w;
        aircraft.state.p;
        aircraft.state.q;
        aircraft.state.r;
        aircraft.state.quat(1);
        aircraft.state.quat(2);
        aircraft.state.quat(3);
        aircraft.state.quat(4);
        aircraft.state.x_e;
        aircraft.state.y_e;
        aircraft.state.z_e
    ];

    control_vec = [
        aircraft.controls.delta_T;
        aircraft.controls.delta_e;
        aircraft.controls.delta_a;
        aircraft.controls.delta_r
    ];

    attitude_vec = [
        aircraft.attitude.phi;
        aircraft.attitude.theta;
        aircraft.attitude.psi
    ];

end

