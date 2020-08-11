function al = al_init(traj_s,n_con)

n_x = size(traj_s.x_bar,1);
n_u = size(traj_s.u_bar,1);
N_tot = size(traj_s.x_bar,2);

al.mu     = [ 1.0.*ones(8,N_tot) ;...
              1.0.*ones(8,N_tot)];
al.lambda = 0.*ones(n_con,N_tot);
al.con    = zeros(n_con,N_tot);
al.con_x  = zeros(n_con,n_x,N_tot);
al.con_u  = zeros(n_con,n_u,N_tot);
al.I_mu   = zeros(n_con,n_con,N_tot);

al.alpha_init = 1;
al.alpha_updt = 0.5;
al.phi_x      = 10;
al.phi_u      = 10;

end