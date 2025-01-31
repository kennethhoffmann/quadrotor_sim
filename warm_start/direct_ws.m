function traj = direct_ws(traj,obj,wts_db,model,nom_show)

% Warm start with an initial run
traj = msl_lqr(1,traj,obj,wts_db,model);

% Publish some diagnostics
switch nom_show
    case 'show'
%         nominal_plot(traj.x,obj,5,'persp');
%         mthrust_debug(traj.u,model)
%         motor_debug(traj.u,model)
    case 'hide'
end

end
