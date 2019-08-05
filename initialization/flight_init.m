function flight = flight_init(model,tf,wp)

act_dt = 1/model.act_hz;
fc_dt  = 1/model.fc_hz;

% Timestamped Actual Pose Array
flight.t_act = 0:act_dt:tf;
flight.x_act = zeros(12,length(flight.t_act));
flight.x_act(:,1) = wp.x(:,1);

% Timestamped FC Pose Array with Cov
flight.t_fc  = 0:fc_dt:tf;
flight.x_fc = zeros(12,length(flight.t_fc));
flight.x_fc(:,1) = wp.x(:,1);

flight.sigma = zeros(12,12,length(flight.t_fc));

% Flight Motor Inputs
flight.m_cmd = zeros(4,length(flight.t_fc)-1);
flight.m_cmd(:,1) = wrench2m_cmd(model.hover_wrench,model);
