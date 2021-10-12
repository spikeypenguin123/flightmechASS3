%% LVLH to body frame 
% 1. Yaw about z-axis of the LVLH frame by angle psi
% 2. Pitch about y-axis of 1st intermediate frame by angle theta
% 3. Roll about x-axis of 2nd intermediate frame by angle phi 
function body = LVLH2body(rpy)
%     rpy(1) = phi;
%     rpy(2) = the;
%     rpy(3) = psi;
    phi = rpy(1);
    the = rpy(2);
    psi = rpy(3);
    
    body = Cx(phi)*Cy(the)*Cz(psi);
end