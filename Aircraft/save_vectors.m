function aircraft = save_vectors(aircraft)
    % Stores the state vectors into the aircraft object
    
    [state_vec, control_vec, attitude_vec] = get_vectors(aircraft);
    
    % extend each vector
    aircraft.vectors.state = [aircraft.vectors.state state_vec];
    aircraft.vectors.control = [aircraft.vectors.control control_vec];
    aircraft.vectors.attitude = [aircraft.vectors.attitude attitude_vec];
end

