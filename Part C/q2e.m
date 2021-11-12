function eulers = q2e(q)   
    % in: 4xn quaternions
    % out: 3xn eulers (rads)

%     % normalise quaternions
%     q = q./sqrt(sum(q.^2,1));
    
    q_0 = q(1,:);
    q_1 = q(2,:);
    q_2 = q(3,:);
    q_3 = q(4,:);
    
    eulers = [
        atan2(2.*(q_0.*q_1+q_2.*q_3),1-2.*(q_1.^2+q_2.^2));
        asin(2.*(q_0.*q_2-q_3.*q_1));
        atan2(2.*(q_0.*q_3+q_1.*q_2),1-2.*(q_2.^2+q_3.^2));
    ];

end