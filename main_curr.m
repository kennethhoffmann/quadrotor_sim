clear; clc; 
addpath(genpath(pwd));

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize Simulation Parameters

% Quadcopter Model
model = model_init('v1.0.0');  

% % % Pre-Computes (comment out after first run to save time)
% dyn_func_init(model,'act');  % Generate Actual Dynamic Functions
% dyn_func_init(model,'est');  % Generate Estimated Dynamic Functions

% lin_func_init('direct',model);
% al_ilqr_init('pid');

n_der = 15;             % Order of Basis Function for QP
qp_init(n_der);         % Generate QP Matrices

% al_ilqr_init('direct');
% al_ilqr_init('pid');

% Objective and Constraints
obj  = obj_init('crescent');
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

% log = simulation(traj,obj,wts_db,model,targ,'df');
log = simulation(traj,map,obj,model,'df','pos_att');
% log = simulation(traj,map,obj,model,targ,'df','open_loop');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Plot the States and Animate

des_err_debug(log);
animation_plot(log,obj,map,'nice','show');
% mthrust_debug(log.u_fmu,model)
