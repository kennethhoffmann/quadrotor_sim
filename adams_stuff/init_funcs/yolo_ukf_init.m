function yukf = yolo_ukf_init(num_dims, dt)
    yukf.dt = dt;
    
    %%% PARAMS %%%
    yukf.prms.b_use_control = false;  % whether to use the control in our estimate
    yukf.b_offset_at_t0 = false;  % whether to add noise to the initial starting location
    %%%% OPTIONS FOR SENSOR %%%%%%%%%%%%%%%%%%%%%%%%
    yukf.prms.b_predicted_bb = true; % true means sensing data comes from predict_quad_bounding_box() instead of from actual yolo data
    
    %%% options for filtering input data
    yukf.prms.b_filter_data = false; % decide if we want to filter data output from yolo (only has an effect if we are using real data, i.e. if b_predicted_bb = false)
    yukf.prms.mv_ave_len_in = 5; % number of samples we use for our moving average filter
    yukf.prms.b_filter_output = false; % decide if we want to filter data output from yolo (only has an effect if we are using real data, i.e. if b_predicted_bb = false)
    yukf.prms.mv_ave_len_out = 5; % number of samples we use for our moving average filter
    
    %%% Options for augmenting our measurement vector
    % Option 1 %%%%%%%   z = [row, col, width, height, angle]
    yukf.prms.b_angled_bounding_box = true; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 2 (DEBUG ONLY) %%%%%%%   z = [state]
    yukf.prms.b_measure_everything = false; % will include a 5th value thats an angle that is rotating the bounding box
    %%%%%%%%%%%%%%%%%%%%
    % Option 3  %%%%%%%   z = [[row, col, width, height, <extra1>, <extra2>, ...]
    yukf.prms.b_measure_aspect_ratio = false; % when not angled, this will include a 5th value (ratio of height to width of bounding box)
    % ___extra A
    yukf.prms.b_measure_yaw = false; % adds the "true" yaw measurement as output of the sensor
    yukf.prms.b_enforce_yaw = false; % this overwrites any dynamics / incorrect update to keep yaw at ground truth value
    yukf.prms.b_enforce_0_yaw = true; % this overwrites any dynamics / incorrect update to keep yaw at 0
    yukf.prms.b_measure_pitch = false;
    yukf.prms.b_enforce_pitch= false; % this overwrites any dynamics / incorrect update to keep pitch at ground truth value
    yukf.prms.b_measure_roll = false;
    yukf.prms.b_enforce_roll = false; % this overwrites any dynamics / incorrect update to keep roll at ground truth value
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    yukf.mu = zeros(num_dims, 1);
    yukf.mu_prev = yukf.mu;
    dim = length(yukf.mu);
    dim_sigma = dim - 1; % because of the quaternion
    
    % these values are in part from prob rob, in part from me choosing
    % them so 1 - alpha^2 + beta = 0, which weight the non-mean sigma
    % points the same as the mean one
    yukf.prms.alpha = .1; % scaling param - how far sig. points are from mean
    yukf.prms.kappa = 2; % scaling param - how far sig. points are from mean
    yukf.prms.beta = 2; % optimal choice according to prob rob

    % these values are used for stepping along sigma directions
    dp = 0.1; % [m]
    dv = 0.005; % [m/s]
    dq = 0.1; % [rad] in ax ang 
    dw = 0.005; % [rad/s]
    yukf.prms.lambda = yukf.prms.alpha^2*(dim_sigma + yukf.prms.kappa) - dim_sigma;
    
    yukf.sigma = diag([dp, dp, dp, dv, dv, dv, dq, dq, dq, dw, dw, dw]);
    assert(length(yukf.sigma) == dim_sigma)
    
    fake_cam.tf_cam_w = eye(4); fake_cam.K = eye(3); fake_cam.tf_cam_ego = eye(4);
    yukf.prms.meas_len = length(predict_quad_bounding_box(yukf.mu, fake_cam, rand(size(init_quad_bounding_box(1,1,1,1))), yukf, yukf.mu));
    yukf.prms.Q = yukf.sigma/10;  % Process Noise
    
    yukf.prms.R = diag([2, 2, 10*ones(1, yukf.prms.meas_len - 2)]); % Measurement Noise
    
    if yukf.prms.b_angled_bounding_box
        yukf.prms.R(5,5) = 0.08;
    	if yukf.prms.b_measure_aspect_ratio
            yukf.prms.R(6,6) = 0.03;
        end
    else
        if yukf.prms.b_measure_aspect_ratio
            yukf.prms.R(5,5) = 0.03;
        end
    end

    yukf.w0_m = yukf.prms.lambda / (yukf.prms.lambda + dim_sigma);
    yukf.w0_c = yukf.w0_m + (1 - yukf.prms.alpha^2 + yukf.prms.beta);
    yukf.wi = 1 / (2*(yukf.prms.lambda + dim_sigma));
    
    yukf.model = model_init('simple vII', 1/yukf.dt, 200, 1000);
end