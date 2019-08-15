function plot_yukf_animation(flight, wp, camera, sv)

    map = wp.map;
    
    t_act = flight.t_act;
    x_act = flight.x_act;
    
    dt = t_act(1,2)-t_act(1,1);

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Define plot window and clear previous stuff
    fig_h = figure(2);
    clf
    [camera_lims, cmr_h] = plot_camera(camera, fig_h); hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Generate flight room map
    gate_h = plot3(map(1,:)',map(2,:)',map(3,:)');
    gate_h.LineWidth = 3;
%     xlim(wp.x_lim);
%     ylim(wp.y_lim);
%     zlim(wp.z_lim);
%     xlim([min(camera_lims(1, 1), wp.x_lim(1)), max(camera_lims(2, 1), wp.x_lim(2))]);
%     ylim([min(camera_lims(1, 2), wp.y_lim(1)), max(camera_lims(2, 2), wp.y_lim(2))]);
%     zlim([min(camera_lims(1, 3), wp.z_lim(1)), max(camera_lims(2, 3), wp.z_lim(2))]);
    xlim([min([camera_lims(1, 1), x_act(1, :) - 1, sv.mu_hist(1, sv.hist_mask) - 1]), max([camera_lims(2, 1), x_act(1, :) + 1, sv.mu_hist(1, sv.hist_mask) + 1])]);
    ylim([min([camera_lims(1, 2), x_act(2, :) - 1, sv.mu_hist(2, sv.hist_mask) - 1]), max([camera_lims(2, 2), x_act(2, :) + 1, sv.mu_hist(2, sv.hist_mask) + 1])]);
    zlim([min([camera_lims(1, 3), x_act(3, :) - 1, sv.mu_hist(3, sv.hist_mask) - 1]), max([camera_lims(2, 3), x_act(3, :) + 1, sv.mu_hist(3, sv.hist_mask) + 1])]);
    grid on
    
    % Set Camera Angle
    daspect([1 1 1])
    view(320,20);
%     view(90,0);

    hold on
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % plot the trajectories
    traj_h = plot3(x_act(1,:), x_act(2,:), x_act(3,:), 'b-');
    traj_est_h = plot3(sv.mu_hist(1,sv.hist_mask), sv.mu_hist(2,sv.hist_mask), sv.mu_hist(3,sv.hist_mask), 'r-');
    
    % Plot the Initial Frame
        % ground truth %%%%%%%%%%%%%%%%%%%
        
        % Body Frame Axes
        vect_x = [0.2 0.0 0.0]';
        vect_y = [0.0 0.2 0.0]';
        vect_z = [0.0 0.0 0.1]';

        % Construct Rotation Matrix
        quat = complete_unit_quat(x_act(7:9, 1));
        bRw = quat2rotm(quat');

        % Determine World Frame Pose of Craft Axes
        pos = x_act(1:3,1);

        x_arrow = [pos pos + (bRw*vect_x)];
        y_arrow = [pos pos + (bRw*vect_y)];
        z_arrow = [pos pos + (bRw*vect_z)];

        x = [x_arrow(1,:); y_arrow(1,:); z_arrow(1,:)]';
        y = [x_arrow(2,:); y_arrow(2,:); z_arrow(2,:)]';
        z = [x_arrow(3,:); y_arrow(3,:); z_arrow(3,:)]';

        % Plot It!
        h_persp = plot3(x, y, z, 'linewidth', 3);

        % Set the Correct Colors
        h_persp(1).Color = [1 0 0];
        h_persp(2).Color = [0 1 0];
        h_persp(3).Color = [0 0 1];
        
        % yukf %%%%%%%%%%%%%%%%%%%
        % Body Frame Axes
        vect_x_est = [0.2 0.0 0.0]';
        vect_y_est = [0.0 0.2 0.0]';
        vect_z_est = [0.0 0.0 0.1]';

        % Construct Rotation Matrix
        quat_est = complete_unit_quat(sv.mu_hist(7:9, 1));
        bRw_est = quat2rotm(quat_est');

        % Determine World Frame Pose of Craft Axes
        pos_est = sv.mu_hist(1:3, 1);

        x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
        y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
        z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];

        x_est = [x_arrow_est(1,:); y_arrow_est(1,:); z_arrow_est(1,:)]';
        y_est = [x_arrow_est(2,:); y_arrow_est(2,:); z_arrow_est(2,:)]';
        z_est = [x_arrow_est(3,:); y_arrow_est(3,:); z_arrow_est(3,:)]';

        % Plot It!
        h_persp_est = plot3(x_est, y_est, z_est, 'linewidth', 3);

        % Set the Correct Colors
        h_persp_est(1).Color = [0.8 0 0];
        h_persp_est(2).Color = [0 0.8 0];
        h_persp_est(3).Color = [0 0 0.8];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % Labels and Legend
    xlabel('x-axis');
    ylabel('y-axis');
    zlabel('z-axis');
    legend([cmr_h; traj_h; traj_est_h; h_persp; h_persp_est], {'camera', 'traj._{gt}', 'traj._{est}', 'X_{gt}','Y_{gt}','Z_{gt}', 'X_{est}','Y_{est}','Z_{est}'});
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Plot the Remainder with REAL-TIME
    curr_time = dt;
    b_real_time = true;
    while (curr_time <= t_act(end))
        tic
        k = ceil(curr_time / dt);
        
        quat = complete_unit_quat(x_act(7:9, k));
        bRw = quat2rotm(quat');
        pos = x_act(1:3, k);
        
        x_arrow = [pos pos + (bRw*vect_x)];
        y_arrow = [pos pos + (bRw*vect_y)];
        z_arrow = [pos pos + (bRw*vect_z)];
        h_persp = reassign(h_persp, x_arrow, y_arrow, z_arrow);
        
        if(sv.hist_mask(k))
            quat_est = complete_unit_quat(sv.mu_hist(7:9, k));
            bRw_est = quat2rotm(quat_est');
            pos_est = sv.mu_hist(1:3, k);

            x_arrow_est = [pos_est pos_est + (bRw_est*vect_x_est)];
            y_arrow_est = [pos_est pos_est + (bRw_est*vect_y_est)];
            z_arrow_est = [pos_est pos_est + (bRw_est*vect_z_est)];
            h_persp_est = reassign(h_persp_est, x_arrow_est, y_arrow_est, z_arrow_est);
        end

        drawnow
        
        if b_real_time
            curr_time = curr_time + toc;
        else
            curr_time = curr_time + dt;
        end
    end
end