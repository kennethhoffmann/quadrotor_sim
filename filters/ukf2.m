    function [mu_new, sigma_new] = ukf2(mu_curr,sigma_curr,u,y,time,model,x_act,quat)
    lambda = 2;
    dim = size(mu_curr,1);
    total_sp = (2*dim+1);
    
    actual = [x_act(1:6,1) ; quat ; x_act(10:12,1)];

    % Unscented Transform 
    sig_points = UT(mu_curr,sigma_curr,dim,lambda,1);

    % Predict %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sigp_pred = zeros(dim,total_sp);
    for k = 1:total_sp
        % Predict Positions
        sigp_pred(1:3,k) = sig_points(1:3,k) + time.fc_dt*mu_curr(4:6,1);

        % Predict Velocities
        vel_dot = lin_acc(sig_points(:,k),u,model,[0 0 0]',0,1);
        sigp_pred(4:6,k) = sig_points(4:6,k) + time.fc_dt*vel_dot;

        % Predict Quaternions
        % omegas
        wx = sig_points(11,k);
        wy = sig_points(12,k);
        wz = sig_points(13,k);

        % Setup Some Useful Stuff for Pred %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        Omega = [ 0 -wx -wy -wz ;...
                 wx   0  wz -wy ;...
                 wy -wz   0  wx ;...
                 wz  wy -wx   0 ];

        q_hat = sig_points(7:10,k) + 0.5*Omega*sig_points(7:10,k)*time.fc_dt;
        sigp_pred(7:10,k) = q_hat./norm(q_hat);

        % Predict Angular Velocities
        omega_dot = ang_acc(u,sig_points(11:13,k),model,[0 0 0]',1);
        sigp_pred(11:13,k) = sig_points(11:13,k) + omega_dot*time.fc_dt;
    end
        
    [mu_ukf_pred, sigma_ukf_pred_bar] = invUT(sigp_pred,dim,lambda,1);
    sigma_ukf_pred = sigma_ukf_pred_bar + model.Q_fil;
    
    % Update
    sigp_upd = UT(mu_ukf_pred,sigma_ukf_pred,dim,lambda,1);
    
    sigp_g = zeros(13,total_sp);  
    for k = 1:total_sp
        sigp_g(:,k) = sensors(time,sigp_upd(:,k),u,model,1);
    end
    
    [~, sigma_y] = invUT(sigp_g,dim,lambda,0);
    mu_y = sigp_g(:,1);
    
    sigma_y = sigma_y + model.R_fil;
    sigma_xy = cross_cov(mu_ukf_pred,sigp_pred,mu_y,sigp_g,dim,lambda);

    mu_new = mu_ukf_pred + sigma_xy*inv(sigma_y)*(y-mu_y);
    mu_new(7:10,1) = mu_new(7:10,1)./norm(mu_new(7:10,1));
    sigma_new = sigma_ukf_pred - sigma_xy*inv(sigma_y)*sigma_xy';
end