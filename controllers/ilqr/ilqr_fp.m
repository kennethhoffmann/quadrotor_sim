function [x_bar,u_bar] = ilqr_fp(x_bar,u_bar,x_now,l,L,alpha,model,Q_t,Q_f,R)
    % Initialize some terms
    N = size(x_bar,2);
    x_fp = zeros(13,N);
    x_fp(:,1) = x_now;
    u_fp = u_bar; 
    cost_curr =  0;
    
for k = 1:N-1
    % Determine Control Command
    del_x = x_fp(:,k)-x_bar(:,k);
    del_u = alpha*l(:,:,k) + L(:,:,k)*del_x;
    u_fp(:,k) = u_fp(:,k) + del_u;

    % Predict Dynamics of Next Step
    FT_ext = zeros(6,1);
    m_cmd = wrench2m_controller(u_fp(:,k),model);

    x_fp(:,k+1) = quadcopter(x_fp(:,k),m_cmd,model,FT_ext,'fc');

    % Update Cost
    cost_curr = cost_curr + 0.5*(del_x'*Q_t*del_x + u_fp(:,k)'*R*u_fp(:,k));
end

% Add terminal cost   
del_x = x_fp(:,N)-x_bar(:,N);
cost_curr = cost_curr + 0.5* del_x'*Q_f*del_x;
%disp(['[ilq_fp]: Current Cost: ',num2str(cost_curr)]);

% If cost goes down, we know it's feasible. Update x_bar.
x_bar = x_fp;
u_bar = u_fp;

end