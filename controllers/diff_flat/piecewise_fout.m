function f_out_full = piecewise_fout(sig_set,fmu_dt,N_dt,n_p)

% Unpack Some Useful Stuff
tf = fmu_dt * (N_dt-1);

mu_r = 1;
mu_angle = 1;
k_r = 4;

OPTIONS = optimoptions('fmincon','Algorithm','sqp','Display','off');
A_sigma = zeros(4,n_p);

for i = 1:4 
    eq_size = k_r+1;
    A_start = zeros(eq_size,n_p);
    A_end   = zeros(eq_size,n_p);
    B_start = zeros(eq_size,1);
    B_end   = zeros(eq_size,1);

    for k = 1:eq_size
        for m = 1:n_p
            if ((k+m-1) <= n_p)
                A_start(k,(k+m-1)) = (0^(m-1))/factorial(m-1);
                A_end(k,(k+m-1))   = (tf^(m-1))/factorial(m-1);
            end
        end
    end
    B_start(1:(k_r+1),1) = sig_set(i,1:5,1)';
    B_end(1:(k_r+1),1)   = sig_set(i,1:5,2)';

    Aeq = [A_start ; A_end];
    Beq = [B_start ; B_end];
    
    % Motor Min/Max Constraint
    A = zeros(0,n_p);
    B = zeros(0,1);
    
    f = @min_func;
    if k == 4
        FUN = @(x)integral(@(t) mu_angle*(f(t,x,n_p))^2,0,tf,'ArrayValued',true);
    else
        FUN = @(x)integral(@(t) mu_r*(f(t,x,n_p))^2,0,tf,'ArrayValued',true);
    end
        
    tic
    A_sigma(i,:) = fmincon(FUN,sig_set(i,:,1)',A,B,Aeq,Beq,[],[],[],OPTIONS);
    toc
end

t_out = linspace(0,tf,N_dt);
f_out_full = zeros(4,n_p,N_dt);

for k = 1:N_dt
    t_kf = t_out(k);
    
    B_sigma = zeros(n_p,n_p);
    for j = 1:n_p
        b_vec = ((t_kf^(j-1))/factorial(j-1))*ones(n_p-j+1,1);
        B_sigma = B_sigma + diag(b_vec,-j+1);
    end
    f_out_full(:,:,k) = A_sigma * B_sigma;
end

end