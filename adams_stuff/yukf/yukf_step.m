function yukf = yukf_step(yukf, u, z, model, camera, initial_bb)
    dim = length(yukf.mu);
    num_sp = 2*dim + 1;
    
    % line 2 prob rob
    sps = calc_sigma_points(yukf.mu, yukf.sigma, yukf); 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 3 prob rob ( Predict ) %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sps_pred = zeros(dim, num_sp);
    for j = 1:num_sp
        sps_pred(1:3,j) = sps(1:3,j) + model.con_dt * yukf.mu(4:6,1);

        % Predict Velocities
        if(~isempty(u))
            vel_dot = lin_acc(sps(:,j), u, model,[0 0 0]', 0, 'actual');
            sps_pred(4:6,j) = sps(4:6,j) + model.con_dt * vel_dot;
        else
            vel_dot = [0;0;0];
            sps_pred(4:6,j) = sps(1:3,j) - yukf.mu_prev(1:3);
        end

        % Predict Quaternions
        % omegas
        wx = sps(10,j);
        wy = sps(11,j);
        wz = sps(12,j);

        % Setup Some Useful Stuff for Pred 
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];
             
        quat = complete_unit_quat(sps(7:9, j)); 
    
        q_hat = quat + 0.5 * Omega * quat * model.con_dt;
        sps_pred(7:9,j) = q_hat(2:4);

        % Predict Angular Velocities
        if(~isempty(u))
            omega_dot = ang_acc(u, sps(10:12, j), model, [0 0 0]', 'actual');
            sps_pred(10:12, j) = sps(10:12, j) + omega_dot * model.con_dt;
        else
            omega_dot = [0;0;0];
            sps_pred(10:12, j) = sps(10:12, j);% - yukf.mu_prev(10:12);
        end
        
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 4 & 5 - predict new mean & cov %%%%%%%%%%%%%%%%%%%%%%%
    [mu_bar, sigma_bar, ~, ~] = predict_mean_and_cov_state(sps_pred, yukf, yukf.prms.R);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 6 - update sigma points 
    sps_updated = calc_sigma_points(mu_bar, sigma_bar, yukf);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 7 - 9 - predict observation & uncertainty for each sigma point
    pred_obs = predict_obs(sps_updated, camera, initial_bb, yukf);
    [z_hat, S, w0_c, wi] = predict_mean_and_cov_obs(pred_obs, yukf, yukf.prms.Q);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 10
    sig_xz = w0_c * (sps_updated(:, 1) - mu_bar) * (pred_obs(:, 1) - z_hat)';
    for i = 2:num_sp
        sig_xz = sig_xz + wi*(sps_updated(:, i) - mu_bar) * (pred_obs(:, i) - z_hat)';
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % line 11 - 13: kalman gain & update
    K = sig_xz * inv(S);
    mu_out = mu_bar + K * (z - z_hat);
    sigma_out = sigma_bar - K * S * K';
    
    % project sigma to pos. def. cone to avoid numeric issues
    [V, D] = eig(sigma_out);
    D(D < 0) = 0.000001;
    sigma_out = V * D * V';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    yukf.mu_prev = yukf.mu;
    yukf.mu = mu_out;
    yukf.sigma = sigma_out;
    disp('')
end