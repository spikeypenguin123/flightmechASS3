function quaternions = e2q(eulers)
    % in: 3xn euler (rad)
    % out: 4xn quat

    % normalise to [0,2pi]
    eulers = eulers+(eulers<0).*(2*pi);
    
    phi = eulers(1,:);
    theta = eulers(2,:);
    psi = eulers(3,:);
    
    quaternions = [
        cos(phi/2).*cos(theta/2).*cos(psi/2) + sin(phi/2).*sin(theta/2).*sin(psi/2);
        sin(phi/2).*cos(theta/2).*cos(psi/2) - cos(phi/2).*sin(theta/2).*sin(psi/2);
        cos(phi/2).*sin(theta/2).*cos(psi/2) + sin(phi/2).*cos(theta/2).*sin(psi/2);
        cos(phi/2).*cos(theta/2).*sin(psi/2) - sin(phi/2).*sin(theta/2).*cos(psi/2)
    ];
end

