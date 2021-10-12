function visualiser = initialise_visualiser(x,y,z,show_displacement,velocity)
    scale_factor = 1;
    load 83plane;
    V=[-V(:,2) V(:,1) V(:,3)];
    V(:,1)=V(:,1)-round(sum(V(:,1))/size(V,1));
    V(:,2)=V(:,2)-round(sum(V(:,2))/size(V,1));
    V(:,3)=V(:,3)-round(sum(V(:,3))/size(V,1));
    correction=max(abs(V(:,1)));
    Vi=V./(scale_factor*correction)*Cx(pi);
    
    figure(1);
    title('Visualisation')
    set(gcf, 'Position',  [100, 100, 500, 500])
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    view(30,45)
    hold on
    anim = Animate('visualisation.mp4','resolution','1000');
    
%     C_LVLH = Cy(pi);
%     LVLH = trplot(C_LVLH,'color','g'); % plot the LVLH frame 
%     Cbv(:,:,1) = LVLH2body(rpy(:,1))*C_LVLH;% initial rotation matrix
%     V=Vi*Cbv(:,:,1);
%     X0=repmat(xyz(:,1)',size(V,1),1);
%     V=V+X0;
%     BODY = trplot(Cbv(:,:,1),'color','r'); %  plot the body frame
%     string= ['Time = ' num2str(t(1))];
%     time_label = annotation('textbox', [0, 0.5, 0, 0], 'string', string);
%     VEHICLE=patch('faces', F, 'vertices' ,V); % plot the surface as a patched together mesh
%     set(VEHICLE, 'facec', [1 1 0]);          
%     set(VEHICLE, 'EdgeColor','none'); lighting phong
    daspect([1 1 1]);
    grid on
    axis equal
    light;
    set(gca,'XTick',[], 'YTick', [], 'ZTick', [])
%     xlim([-1000,1000]);
%     ylim([-1000,1000]);
%     zlim([-1000,1000]);

    annotation('textbox', [0, 0.25, 0, 0], 'string', "Displacement not to scale.");

    visualiser.anim = anim;
    visualiser.vehicle = Vi;
    visualiser.F = F;
    visualiser.x_0 = [x y z];
    visualiser.trace = [animatedline animatedline animatedline];
    visualiser.show_displacement = show_displacement;
    
    % factor used to make the plots more readable
    visualiser.velocity_factor = velocity*10;
end

