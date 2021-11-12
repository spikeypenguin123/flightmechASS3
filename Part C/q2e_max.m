function eulerangles = q2e_max(quaternions)

% Normalising the quaternions
norm = (quaternions(1,:).^2 + quaternions(2,:).^2 + quaternions(3,:).^2 + quaternions(4,:).^2).^0.5;
% Converting Quaternions to Euler Angles
% Quaternion inputs
for i = 1:length(norm)
    q0 = quaternions(1,:)/norm(i);
    q1 = quaternions(2,:)/norm(i);
    q2 = quaternions(3,:)/norm(i);
    q3 = quaternions(4,:)/norm(i);
end

% Converting quaternion inputs to corresponding Euler Angles
phi = atan2((q2.*q3 + q0.*q1),(q0.^2 + q3.^2 - 0.5));
theta = atan2((q0.*q2 - q1.*q3),sqrt((q0.^2 + q1.^2 - 0.5).^2 + (q1.*q2 + q0.*q3).^2));
psi = atan2((q1.*q2 + q0.*q3),(q0.^2 + q1.^2 - 0.5));

% Outputing Euler Angles in degrees
eulerangles = rad2deg([phi ; theta ; psi]);

end