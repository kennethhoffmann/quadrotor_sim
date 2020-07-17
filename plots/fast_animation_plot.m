function fast_animation_plot(x_act,wp,view_point)
    % Reset window
    figure(2)
    clf
    grid on
    hold on
    set(gcf,'color','white')
    daspect([1 1 1])
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    
    % Define Map Limits
    xlim(wp.x_lim);
    ylim(wp.y_lim);
    zlim(wp.z_lim);

    % Generate gate
    gate_h = plot3(wp.p_gate(1,:)',wp.p_gate(2,:)',wp.p_gate(3,:)');
    gate_h.LineWidth = 3;
    
    % Generate waypoints
    for k = 1:size(wp.x,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(wp.x(:,k));
        x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
        y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
        z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

        h_wp = plot3(x,y,z,'linewidth',2);

        % Set the Correct Colors
        h_wp(1).Color = [1 0 0];
        h_wp(2).Color = [0 1 0];
        h_wp(3).Color = [0 0 1];
    end

    % Set Camera Angle
    switch view_point
        case 'persp'
            view(320,20);
        case 'back'
            view(-90,0);
        case 'side'
            view(0,0);
    end
    
    % Plot the full trajectory
    plot3(x_act(1,:),x_act(2,:),x_act(3,:),'--k','linewidth',1);
        
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Initial Frame  
    [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,1));
    x = [x_arrow(1,:) ; y_arrow(1,:) ; z_arrow(1,:)]';
    y = [x_arrow(2,:) ; y_arrow(2,:) ; z_arrow(2,:)]';
    z = [x_arrow(3,:) ; y_arrow(3,:) ; z_arrow(3,:)]';

    h_persp = plot3(x,y,z,'linewidth',3);
    
    % Set the Correct Colors
    h_persp(1).Color = [1 0 0];
    h_persp(2).Color = [0 1 0];
    h_persp(3).Color = [0 0 1];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Animate the trajectory
    for k = 1:10:size(x_act,2)
        [x_arrow, y_arrow, z_arrow] = frame_builder(x_act(:,k));
        h_persp = reassign(h_persp,x_arrow,y_arrow,z_arrow);
        
        drawnow
        pause(0.1);
    end
end
