function [traj,log] = trajGenVel(start,goal,endPos,plot)
%clear; clc; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Quadcopter Model
model = model_init('v1.0.0');  

n_der = 15;             % Order of Basis Function for QP
% qp_init(n_der);        % Generate QP Matrices

% Objective and Constraints
%obj  = obj_init('target');
%start  = [-4,0,1]';
%goal   = [-1,0,1.2]';
%endPos = [4,0,1]';
%Does not incorporate the orientation yet
obj  = obj_init('target',start,goal,endPos);
map  = map_init('default');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Warm Start
tic
 
% Diff. Flat Warm Start
traj = diff_flat_ws(obj,map,model,n_der,'hide');

% % Full Constraint Resolution
% traj = al_ilqr(traj,obj,map,model);

toc
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulation

log = simulation(traj,obj,model,'none','pos_att','bypass');
% log = simulation(traj,obj,model,'none','body_rate','bypass');
% log = simulation(traj,obj,model,'none','direct','bypass');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate
if plot
animation_plot(log,obj,map,'persp','show');
end
plot3(traj.x(1,:),traj.x(2,:),traj.x(3,:))
axis equal
axis([map.x_lim,map.y_lim,map.z_lim])

plotfixer
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Save the Flat Output in Pos/Vel csv

fout2csv(log.t_fmu,traj.f_out)

end
