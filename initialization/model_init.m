function model = model_init(mdl_type,ctl_type)

model.g = 9.81;

hz_est = 200;       % State Estimator Sample Rate
hz_lqr = 5;         % iLQR Update Rate
hz_fbc = 200;       % Feedback Controller Rate
hz_act = 1000;      % Actual Dynamics Update Rate

switch ctl_type     % Control Law Update Rate
    case 'high-speed'
        hz_ctl = 200;
    case 'low-speed'
        hz_ctl = 20;
end

switch mdl_type
    case 'simple v0.0'              % simple motor, no noise, no drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%          
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];  
        model.kt_est = [1.5683e-6 0.00 0.00]';
        model.b_est  = 0.05; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];            
        model.kt_act = [1.5683e-6 0.00 0.00]';
        model.b_act  = 0.05;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'simple v0.2'               % simple motor, with noise, no drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'simple v0.4'               % simple motor, with noise, with drag
        disp('[model init]: || [ ] Full Quadratic Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.5683-06 ; 0 ; 0];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.5683e-06 ; 0 ; 0];
        model.b_act  = 0.0011;
        model.kd_act = 0.1;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'simple v0.6'               % full quadratic motor, with noise, no drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [ ] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        % Model Noise
        W_pos   = 0.0*ones(3,1);
        W_vel   = 0.0*ones(3,1);
        W_quat  = 0.0*ones(4,1);
        W_omega = 0.0*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'simple v0.8'              % full quadratic motor, with noise, with drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [*] Process Noise || [ ] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.0;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.0;
        model.L_act  = 0.0885;
        % Model Noise
        % Model Noise
        W_pos   = 0.001*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.001*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
    case 'simple v1.0'              % full quadratic motor, with noise, with drag
        disp('[model init]: || [*] Full Quadratic Motor Model || [*] Process Noise || [*] Drag ||');
        % Estimate %%%
        model.m_est = 0.650;
        model.I_est = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];    
        model.kt_est = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_est  = 0.0011; 
        model.kd_est = 0.1;
        model.L_est  = 0.0885;
        
        % Actual %%%
        model.m_act = 0.650;
        model.I_act = 0.0001.*[  2.14   0.00   0.00;...
                                 0.00   2.14   0.00;...
                                 0.00   0.00  42.00];             
        model.kt_act = [1.8512e-6 ; -5.6076e-4 ; 0.2210];
        model.b_act  = 0.0011;
        model.kd_act = 0.1;
        model.L_act  = 0.0885;
        % Model Noise
        W_pos   = 0.001*ones(3,1);
        W_vel   = 0.001*ones(3,1);
        W_quat  = 0.001*ones(4,1);
        W_omega = 0.001*ones(3,1);
        model.W = diag([W_pos ; W_vel ; W_quat ; W_omega]);
end

model.leg_l = 0.15;

model.inv_I_est = inv(model.I_est);
model.inv_I_act = inv(model.I_act);

L = model.L_est;
b = model.b_est;
model.motor2wrench = [ 1  1  1  1;...
                      -L  L  L -L;...
                      -L  L -L  L;...
                      -b -b  b  b];

if det(model.W) == 0
    model.W_inv = model.W;
else
    model.W_inv = inv(model.W);
end

model.motor_min = 400;      % Motor Min rad/s
model.motor_max = 4800;     % Motor Max rad/s

model.hz_est = hz_est;
model.dt_est = 1/hz_est;

model.hz_lqr = hz_lqr;
model.dt_lqr = 1/hz_lqr;

model.hz_ctl = hz_ctl;
model.dt_ctl = 1/hz_ctl;

model.hz_fbc = hz_fbc;
model.dt_fbc = 1/hz_fbc;

model.hz_act = hz_act;
model.dt_act = 1/hz_act;

disp('[model init]: A_calc and B_calc uses a kw^2 approx.');
disp('[model init]: Thrust to Wrench uses L_est and b_est');
disp('[model init]: Leg length set to 0.2m');