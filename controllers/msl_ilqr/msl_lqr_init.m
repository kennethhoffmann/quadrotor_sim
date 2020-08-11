function traj = msl_lqr_init(N_seg,obj,model)
    % wp for waypoints
    % seg for segments
    % tot for total
    
    N_wp = size(obj.wp_arr,2)-1;
    N_tot = (N_wp * (N_seg-1)) + 1;
    
    traj.kf_seg = 1:(N_seg-1):N_tot;
    
    traj.wp_arr = obj.wp_arr;
    
    traj.u_bar = model.hover_u.*ones(4,N_tot-1);
    traj.x_bar = zeros(13,N_tot);
    traj.x_bar(:,1) = obj.wp_arr(:,1);
    
    traj.l = zeros(4,1,N_tot-1);
    traj.L = zeros(4,13,N_tot-1);
    
    traj.J     = 99999.*ones(1,N_wp);
    traj.J_stg = 99999.*ones(1,N_tot);
    traj.J_aug = 99999.*ones(1,N_tot);
end