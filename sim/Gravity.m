function G_body = Gravity(g,q,m)
    % in: gravitational constant g (m/s^2), 4x1 quaternions q, mass m (kg)
    % out: 3x1 G_body (N)
    
    G_body = g*m*[0 0 1]*DCM(q)';
    
end
