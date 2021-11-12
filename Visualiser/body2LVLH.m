%% LVLH to body frame 
% 1. Yaw about z-axis of the LVLH frame by angle psi
% 2. Pitch about y-axis of 1st intermediate frame by angle theta
% 3. Roll about x-axis of 2nd intermediate frame by angle phi 
function LVLH = body2LVLH(pqr)
    LVLH = Cz(-pqr(3))*Cy(-pqr(2))*Cx(-pqr(1));
end