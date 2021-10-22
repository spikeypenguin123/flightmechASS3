function aircraft = PushState(X,U,aircraft)
    % Pushing the state variables 
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
    
    % Pushing the control variables
    aircraft.controls.delta_T = U(1);   % Thrust    (Newtons)
    aircraft.controls.delta_e = U(2);   % Elevator  (rad)
end