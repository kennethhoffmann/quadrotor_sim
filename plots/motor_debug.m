function motor_debug(u_sim,model)
    figure(4)
    clf

    kt = model.kt_est(1,1);
    
    points = size(u_sim,2);
    omega_m = zeros(4,points);
    
    motor_min = model.motor_min .* ones(1,points);
    motor_max = model.motor_max .* ones(1,points);
    
    for k = 1:points
        f_m = motor_transform(u_sim(:,k),model,'sim');
        
        omega_m(:,k) = sign(f_m).*sqrt((1/kt).*abs(f_m));
    end
   
    for k = 1:4
        subplot(3,2,k)
        plot(omega_m(k,:))
        hold on
        
        plot(motor_min,'--')
        plot(motor_max,'--')
        xlabel('Time(s)');
        ylabel('\omega_{m} (rad s^{-1})');
        ylim([0 3500]);
    end
    
    subplot(3,2,5)
    plot(u_sim(2,:))
    xlabel('Time(s)');
    ylabel('\tau_x (Nm)');
    ylim([-0.02 0.02]);
    
    subplot(3,2,6)
    plot(u_sim(3,:))
    xlabel('Time(s)');
    ylabel('\tau_y (Nm)');
    ylim([-0.1 0.1]);

end