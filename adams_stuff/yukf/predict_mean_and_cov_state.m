function [mu_bar, sig_bar] = predict_mean_and_cov_state(sps, yukf, cov_add_noise)
    global k
    % inverse of the unscented transform
    % see https://kodlab.seas.upenn.edu/uploads/Arun/UKFpaper.pdf for quat stuff
    
    % Calculate mean    
    mu_bar = (yukf.w0_m * sps(:, 1)) + (yukf.wi*sum(sps(:, 2:end), 2)); % calculate the mean of the vector parts
    [mu_bar(7:10), ei_vec_set] = calc_mean_quat(sps(7:10, :)', yukf); % now the quaternion parts
%     [sps(7:10, :)'; mu_bar(7:10)]
    
    % Calc covar (take into account quats!) - note ei_vec_set was calculated 
    % for the mean, but also can be used for the covar
    sig_bar = calc_covar_quat(sps, mu_bar, yukf, ei_vec_set, cov_add_noise);
end