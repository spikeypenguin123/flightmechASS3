function PlotData(vectors,t)
    
    % Settings 
    posx = 10;
    posy = 50;
    lenx = 800;
    leny = 400;
    figsize1 = [posx posy+leny*1.25 lenx leny];
    figsize2 = [posx+lenx posy+leny*1.25 lenx leny];
    figsize3 = [posx posy lenx leny];
    figsize4 = [posx+lenx posy lenx leny];
    figsize5 = [posx posy 400, 400];
    
    % Extract Data 
    X = vectors.state;
    U = vectors.control;
    
    % x,y,z position
%     plot(t,vectors.state(13,:));
    
    % Plot velocity (uvw)
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize1)
    fig2 = figure(2);
    subplot(3,1,1)
    plot(t,X(1,:),'k-','LineWidth',2);
    grid minor
    title('u','Interpreter','Latex');
    subplot(3,1,2)
    plot(t,X(2,:),'k-','LineWidth',2);
    grid minor
    title('v','Interpreter','Latex');
    subplot(3,1,3)
    plot(t,X(3,:),'k-','LineWidth',2);
    grid minor
    title('w','Interpreter','Latex');
    han=axes(fig2,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel(han,'Time, (s)','Interpreter','Latex');
    ylabel(han,'Velocity, (m/s)','Interpreter','Latex');
    
    % Plot body rates (pqr) 
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize2)
    fig3 = figure(3);
    subplot(3,1,1)
    plot(t,rad2deg(X(4,:)),'k-','LineWidth',2);
    grid minor
    title('p','Interpreter','Latex');
    subplot(3,1,2)
    plot(t,rad2deg(X(5,:)),'k-','LineWidth',2);
    grid minor
    title('q','Interpreter','Latex');
    subplot(3,1,3)
    plot(t,rad2deg(X(6,:)),'k-','LineWidth',2);
    grid minor
    title('r','Interpreter','Latex');
    han=axes(fig3,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('Body Rate, (deg/s)','Interpreter','Latex');

    
    % Plot attitude (phi,the,psi)
    euler = rad2deg(q2e(X(7:10,:))); % convert the quaternions to euler
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize3)
    fig4 = figure(4);
    subplot(3,1,1)
    plot(t,euler(1,:),'k-','LineWidth',2);
    grid minor
    title('\phi','FontName','Latex');
    subplot(3,1,2)
    plot(t,euler(2,:),'k-','LineWidth',2);
    grid minor
    title('\theta','FontName','Latex');
    subplot(3,1,3)
    plot(t,euler(3,:),'k-','LineWidth',2);
    grid minor
    title('\psi','FontName','Latex');
    han=axes(fig4,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('Attitude, (deg)','Interpreter','Latex');
    
    % Plot position, Earth frame (xyz)
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize4)
    fig5 = figure(5);
    subplot(3,1,1)
    plot(t,X(11,:),'k-','LineWidth',2);
    grid minor
    title('x_{e}','FontName','Latex');
    subplot(3,1,2)
    plot(t,X(12,:),'k-','LineWidth',2);
    grid minor
    title('y_{e}','FontName','Latex');
    subplot(3,1,3)
    plot(t,-X(13,:),'k-','LineWidth',2);
    grid minor
    title('Altitudem (-z_{e})','FontName','Latex');
    han=axes(fig5,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('Position, (m)','Interpreter','Latex');
    
    
    % Plot position, Earth frame (xyz)
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize5)
    fig6 = figure(6);
    subplot(2,2,1)
    plot(t,U(1,:),'k-','LineWidth',2);
    grid minor
    title('Throttle','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{T}, (fraction)','FontName','Latex');
    subplot(2,2,2)
    plot(t,U(2,:),'k-','LineWidth',2);
    grid minor
    title('Elevator','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{e}, (deg)','FontName','Latex');
    subplot(2,2,3)
    plot(t,U(3,:),'k-','LineWidth',2);
    grid minor
    title('Aileron','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{a}, (deg)','FontName','Latex');
    subplot(2,2,4)
    plot(t,U(4,:),'k-','LineWidth',2);
    grid minor
    title('Rudder','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{r}, (deg)','FontName','Latex');
   
    % Save Figures to a folder
%     if saveMode
%         
%     end
    
end

