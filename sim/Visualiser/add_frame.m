function vis = add_frame(vis, x, y, z, phi, theta, psi, t)

    figure(1)
    
    if t > 0
        if ~vis.show_displacement
            delete(vis.BODY); % remove the old frame
        end
        delete(vis.time_label); % remove old time
        delete(vis.VEHICLE);   % remove vehicle
    end
    
    Cbv = Cx(-pi-phi)*Cy(-theta)*Cz(psi);
    Vnew=vis.vehicle*Cbv;
    
    if vis.show_displacement
        delta_x = ([x y -z]-vis.x_0)./vis.velocity_factor;
    else
        delta_x = [0 0 0];
    end
    
    X0=repmat(delta_x,size(Vnew,1),1);
    Vnew=Vnew+X0;
    
    if ~vis.show_displacement
        vis.BODY = trplot(Cbv','color','r'); %  plot the body frame
    end
    vis.time_label = annotation('textbox', [0, 0.5, 0, 0], 'string', "Time: " + t); % display time on plot
    vis.VEHICLE=patch('faces', vis.F, 'vertices' ,Vnew); % plot the surface as a patched together mesh
    set(vis.VEHICLE, 'facec', [1 1 0]);          
    set(vis.VEHICLE, 'EdgeColor','none'); 
    
    % add a point on each wingtip and in centre
    rightwing = [0 0.95 -0.2]*Cbv+delta_x;
    leftwing = [0 -0.95 -0.2]*Cbv+delta_x;
    addpoints(vis.trace(1),rightwing(1),rightwing(2),rightwing(3))
    addpoints(vis.trace(2),leftwing(1),leftwing(2),leftwing(3))
    addpoints(vis.trace(3),delta_x(1),delta_x(2),delta_x(3))
    
    axis equal
    
    vis.anim.add(); % add frame to animation 
end


