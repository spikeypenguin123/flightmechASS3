% vehicle test with LVLH and Body axes

close all 
clear all
clc

lat = deg2rad(0);
lon = deg2rad(0);
%% State Variables
% Roll:     [-pi,   pi]
% Pitch:    [-pi/2, pi/2]
% Yaw:      [0,     2pi]

% State vector 
%    position  velocity  attitude     ang vel
%   [xe,ye,ze, ub,vb,wb, phi,the,psi, pb,qb,rb]
%    posEARTH  uvwBODY   rpyLVLH      pqrBODY      

% Position (BODY)
xyz = [0;0;0];              % m
% Velocity (BODY)
uvw = [0;0;0];              % m/s
% Attitude (LVLH)
rpy = deg2rad([0;0;90]);   % rad
% Angular Velocity (BODY)
pqr = deg2rad([30;20;-20]);    % rad/s 

% Convert velocity to LVLH (ensures the object moves in correct direction)
uvw = body2LVLH(rpy(:,1))*uvw;
uvw(2) = -uvw(2);
uvw(3) = -uvw(3);

%% Vehicle Variables
scale_factor = 1;
load 83plane;
V=[-V(:,2) V(:,1) V(:,3)];
V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
correction=max(abs(V(:,1)));
Vi=V./(scale_factor*correction)*Cx(pi);

%% Time vector
t_start = 0;
t_end = 10;
step = 0.1;
t_elements = (t_end-t_start)/step;
t = linspace(t_start,t_end,t_elements);
t(1) = 0;

%% Plot the figure at initial conditions
figure(1)
title('LVLH Grid')
set(gcf, 'Position',  [100, 100, 500, 500])
xlabel('X');
ylabel('Y');
zlabel('Z');

% set(gca, 'YDir','reverse')
view(30,45)
% ECEF = trplot(); % plot the ECEF frame 
hold on
% LVLH = trplot(Cz(-rpy(3))*Cy(rpy(2))*Cx(rpy(1)),'color','m'); % plot the LVLH frame 
% C_NED = NED2ECEFseq2(lat,lon)';
% NED = trplot(C_NED,'color','m'); % plot the NED frame 
C_LVLH = Cy(pi);
LVLH = trplot(C_LVLH,'color','g'); % plot the LVLH frame 
Cbv(:,:,1) = LVLH2body(rpy(:,1))*C_LVLH;% initial rotation matrix
V=Vi*Cbv(:,:,1);
X0=repmat(xyz(:,1)',size(V,1),1);
V=V+X0;
BODY = trplot(Cbv(:,:,1),'color','r'); %  plot the body frame
string= ['Time = ' num2str(t(1))];
time_label = annotation('textbox', [0, 0.5, 0, 0], 'string', string);
VEHICLE=patch('faces', F, 'vertices' ,V); % plot the surface as a patched together mesh
set(VEHICLE, 'facec', [1 1 0]);          
set(VEHICLE, 'EdgeColor','none'); lighting phong
daspect([1 1 1]);
grid on
light;
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,1]);

% Begin Animation
anim = Animate('attitude_visualisation.mp4','resolution','1000');

% Time dependant visualisation/animation running through loop
for i = 1:t_elements-1
    tic % start timing how long this part of loop goes for 
    
    % force calc for rates (constant for this example)
    uvw(:,i+1) = uvw(:,i);
    pqr(:,i+1) = pqr(:,i);
    
    % euler integration
    xyz(:,i+1) = xyz(:,i) + uvw(:,i)*step; 
    rpy(:,i+1) = rpy(:,i) + pqr(:,i)*step;   % euler int of next attitude coord
    Cbv(:,:,i+1) = LVLH2body(rpy(:,i+1))*C_LVLH;
%     Vnew=V*Cbv(:,:,i+1);
    Vnew=Vi*Cbv(:,:,i+1);
    X0=repmat(xyz(:,i+1)',size(Vnew,1),1);
    Vnew=Vnew+X0;

    
    % Update figure
    figure(1)
    delete(BODY); % remove the old frame
    delete(time_label); % remove old time
    delete(VEHICLE);   % remove vehicle
    BODY = trplot(Cbv(:,:,i+1)','color','r'); %  plot the body frame
    string= ['Time = ' num2str(t(i+1))];
    time_label = annotation('textbox', [0, 0.5, 0, 0], 'string', string); % display time on plot
    VEHICLE=patch('faces', F, 'vertices' ,Vnew); % plot the surface as a patched together mesh
    set(VEHICLE, 'facec', [1 1 0]);          
    set(VEHICLE, 'EdgeColor','none'); 
    
    anim.add(); % add frame to animation 
    tim = toc; % computation time for this part of the loop
    if step-tim > 0
        pause(step-tim); % step time minus the computation time
    end
end
anim.close(); 

figure(2)
set(gcf, 'Position',  [2000, 100, 1000, 1200])
subplot(4,1,1)
plot(t,xyz(1,:));
hold on
grid on 
plot(t,xyz(2,:));
plot(t,xyz(3,:));
title('Position, (EARTH)')
xlabel("Time, (s)")
ylabel("Position, (m)")
legend("x_{e}","y_{e}","z_{e}",'FontName','Latex');

subplot(4,1,2)
plot(t,uvw(1,:));
hold on
grid on 
plot(t,uvw(2,:));
plot(t,uvw(3,:));
title('Velocity, (BODY)')
xlabel("Time, (s)")
ylabel("Velocity, (m/s)")
legend("u_{b}","v_{b}","w_{b}",'FontName','Latex');

subplot(4,1,3)
plot(t,rpy(1,:));
hold on
grid on 
plot(t,rpy(2,:));
plot(t,rpy(3,:));
title('Attitude, (LVLH)')
xlabel("Time, (s)")
ylabel("Euler angles, (Rad)")
legend("Roll, \phi","Pitch, \theta","Yaw, \psi",'FontName','Latex');

subplot(4,1,4)
plot(t,pqr(1,:));
hold on
grid on 
plot(t,pqr(2,:));
plot(t,pqr(3,:));
title('Angular Velocity, (BODY)')
xlabel("Time, (s)")
ylabel("Angular Velocity, (Rad/s)")
legend("p_{b}","q_{b}","r_{b}",'FontName','Latex');

