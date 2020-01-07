function [l,L] = ilqr_bp(x_itr,x_bar,u_bar,A,B,Q_t,Q_f,R)

v = Q_f*(x_bar(:,end)-x_itr(:,end));
V = Q_f;

% Execute the Backward Pass
N = size(x_bar,2);
for k = N-1:-1:1
    % Update the Stagewise Variables
    Q_x  = Q_t *(x_bar(:,k)-x_itr(:,k)) + A(:,:,k)'*v;
    Q_xx = Q_t + A(:,:,k)'*V*A(:,:,k);
    Q_u  = R*u_bar(:,k) + B(:,:,k)'*v;
    Q_uu = R + B(:,:,k)'*V*B(:,:,k);
    Q_ux = B(:,:,k)'*V*A(:,:,k);
    
    % Update the feed-forward and feedback terms
    l(:,:,k) = -(Q_uu+eye(4))\Q_u;
    L(:,:,k) = -(Q_uu+eye(4))\Q_ux;

    % Update v and V for next bp state
    v = Q_x - L(:,:,k)'*Q_uu*l(:,k);
    V = Q_xx - L(:,:,k)'*Q_uu*L(:,:,k);
end

end