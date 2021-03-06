function PlotData(vectors,t,filename,save)
    
    % Settings 
    posx = 10;
    posy = 50;
    lenx = 650;
    leny = 400;
    figsize1 = [posx posy+leny*1.25 lenx leny*1.5];
    figsize2 = [posx+lenx posy+leny*1.25 lenx leny];
    figsize3 = [posx posy lenx leny];
    figsize4 = [posx+lenx posy lenx leny];
    figsize5 = [posx+(lenx*0.5) posy+(leny*0.5) lenx, leny*1.1];
    
    % Extract Data 
    X = vectors.state;
    U = vectors.control;
    
    % Calculate the velocity
    V = sqrt(X(1,:).^2 + X(2,:).^2 + X(3,:).^2);
    
    % x,y,z position
%     plot(t,vectors.state(13,:));
    
    % Plot velocity (uvw)
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize1)
    fig2 = figure(2);
    subplot(4,1,1)
    plot(t,X(1,:),'k-','LineWidth',2);
    grid minor
    title('u','Fontname','Latex');
    subplot(4,1,2)
    plot(t,X(2,:),'k-','LineWidth',2);
    grid minor
    title('v','Fontname','Latex');
    subplot(4,1,3)
    plot(t,X(3,:),'k-','LineWidth',2);
    grid minor
    title('w','Fontname','Latex');
    subplot(4,1,4)
    plot(t,V,'k-','LineWidth',2);
    grid minor
    title('V','Fontname','Latex');
    han=axes(fig2,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel(han,'Time, (s)','Interpreter','Latex');
    ylabel(han,'Velocity, (m/s)','Interpreter','Latex');
    if save == 1
        saveas(gcf,['velocity_',filename,'.png']);
    end
    
    % Plot body rates (pqr) 
    figure('DefaultAxesFontSize',16,'Renderer', 'painters', 'Position', figsize2)
    fig3 = figure(3);
    subplot(3,1,1)
    plot(t,rad2deg(X(4,:)),'k-','LineWidth',2);
    grid minor
    title('p','Fontname','Latex');
    subplot(3,1,2)
    plot(t,rad2deg(X(5,:)),'k-','LineWidth',2);
    grid minor
    title('q','Fontname','Latex');
    subplot(3,1,3)
    plot(t,rad2deg(X(6,:)),'k-','LineWidth',2);
    grid minor
    title('u','Fontname','Latex');
    han=axes(fig3,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel(han,'Time, (s)','Interpreter','Latex');
    ylabel(han,'Velocity, (m/s)','Interpreter','Latex');
    if save == 1
        saveas(gcf,['bodyrates_',filename,'.png']);
    end

    
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
    if save == 1
        saveas(gcf,['attitude_',filename,'.png']);
    end
    
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
    title('Altitude (-z_{e})','FontName','Latex');
    han=axes(fig5,'visible','off'); 
    han.XLabel.Visible='on';
    han.YLabel.Visible='on';
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('Position, (m)','Interpreter','Latex');
    if save == 1
        saveas(gcf,['position_',filename,'.png']);
    end
    
    
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
    plot(t,rad2deg(U(2,:)),'k-','LineWidth',2);
    grid minor
    title('Elevator','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{e}, (deg)','FontName','Latex');
    subplot(2,2,3)
    plot(t,rad2deg(U(3,:)),'k-','LineWidth',2);
    grid minor
    title('Aileron','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{a}, (deg)','FontName','Latex');
    subplot(2,2,4)
    plot(t,rad2deg(U(4,:)),'k-','LineWidth',2);
    grid minor
    title('Rudder','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\delta_{r}, (deg)','FontName','Latex');
    if save == 1
        saveas(gcf,['controls_',filename,'.png']);
    end
   
    
    % plot vertical g-force
    figure('DefaultAxesFontSize',16,'Renderer', 'painters')
    fig7 = figure(7);
    plot(t, X(5,:).*X(1,:)/9.81 + 1, 'k-', 'LineWidth',2);
    grid minor
%     title('Vertical G-Force','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('G-Force','FontName','Latex');
    if save == 1
        saveas(gcf,['gforce_',filename,'.png']);
    end
    
    figure('DefaultAxesFontSize',16,'Renderer', 'painters')
    fig8 = figure(8);
    plot(t, atan2d(X(3,:),X(1,:)), 'k-', 'LineWidth',2); % was v/u but fixed it to w/u
    grid minor
%     title('Angle of Attack','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('AoA (deg)','FontName','Latex');
    if save == 1
        saveas(gcf,['AoA_',filename,'.png']);
    end
    
    figure('DefaultAxesFontSize',16,'Renderer', 'painters')
    fig9 = figure(9);
    plot(t, asind(X(2,:)./V), 'k-', 'LineWidth',2); % asin(v/V)
    grid minor
%     title('Sideslip, \beta','FontName','Latex');
    xlabel('Time, (s)','Interpreter','Latex');
    ylabel('\beta (deg)','FontName','Latex');
    
    
    % Save Figures to a folder
%     if saveMode
%         
%     end
    
end

