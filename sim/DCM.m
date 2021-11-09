function output = DCM(quat)
%% Inputs 
% rotation: [q0, q1, q2. q3]^T 
%% Outputs the Direct Cosine Matrix for this attitude in quats
q0 = quat(1);  % w component, (real part)
q1 = quat(2);  % i component
q2 = quat(3);  % y component
q3 = quat(4);  % z component

% l1 = q0^2 + q1^2 + q2^2 + q3^2;  %
l1 = q0^2 + q1^2 - q2^2 - q3^2;  %
l2 = 2*(q1*q2 + q0*q3);
l3 = 2*(q1*q3 - q0*q2);
m1 = 2*(q1*q2 - q0*q3);
% m2 = q0^2 - q1^2 + q2^2 + q3^2; % 
m2 = q0^2 - q1^2 + q2^2 - q3^2; % 
m3 = 2*(q2*q3 + q0*q1);
n1 = 2*(q0*q2 + q1*q3);
n2 = 2*(q2*q3 - q0*q1);
n3 = q0^2 - q1^2 - q2^2 + q3^2;

output = [l1, l2, l3; m1, m2, m3; n1, n2, n3];

end