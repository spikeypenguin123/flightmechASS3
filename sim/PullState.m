
function X = PullState(aircraft)
    X = zeros(13,1);                   % Initialise X0
    X(1)   = aircraft.state.u;         % (m/s)       
    X(2)   = aircraft.state.v;         % (m/s)       
    X(3)   = aircraft.state.w;         % (m/s)       
    X(4)   = aircraft.state.p;         % (rad/s)     
    X(5)   = aircraft.state.q;         % (rad/s)     
    X(6)   = aircraft.state.r;         % (rad/s)     
    X(7)   = aircraft.state.quat(1);   % quat w     
    X(8)   = aircraft.state.quat(2);   % quat x      
    X(9)   = aircraft.state.quat(3);   % quat y      
    X(10)  = aircraft.state.quat(4);   % quat z 
    X(11)  = aircraft.state.x_e;       % (m)         
    X(12)  = aircraft.state.y_e;       % (m)         
    X(13)  = aircraft.state.z_e;       % (m)     
end