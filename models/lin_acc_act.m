function v_dot = lin_acc_act(F_ext_x,F_ext_y,F_ext_z,q_w,q_x,q_y,q_z,u1,u2,u3,u4,v_x,v_y,v_z)
%LIN_ACC_ACT
%    V_DOT = LIN_ACC_ACT(F_EXT_X,F_EXT_Y,F_EXT_Z,Q_W,Q_X,Q_Y,Q_Z,U1,U2,U3,U4,V_X,V_Y,V_Z)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    06-May-2021 17:49:37

t2 = q_w.^2;
t3 = q_x.^2;
t4 = q_y.^2;
t5 = q_z.^2;
t6 = u1.^2;
t7 = u2.^2;
t8 = u3.^2;
t9 = u4.^2;
t10 = q_w.*q_x.*2.0;
t11 = q_w.*q_y.*2.0;
t12 = q_w.*q_z.*2.0;
t13 = q_x.*q_y.*2.0;
t14 = q_x.*q_z.*2.0;
t15 = q_y.*q_z.*2.0;
t21 = (q_w.*q_x)./5.0e+8;
t22 = (q_w.*q_y)./5.0e+8;
t23 = (q_w.*q_z)./5.0e+8;
t24 = (q_x.*q_y)./5.0e+8;
t25 = (q_x.*q_z)./5.0e+8;
t26 = (q_y.*q_z)./5.0e+8;
t16 = -t11;
t17 = -t13;
t18 = -t2;
t19 = -t3;
t20 = -t5;
t27 = t2./1.0e+9;
t28 = t3./1.0e+9;
t29 = t4./1.0e+9;
t30 = t5./1.0e+9;
t41 = t6.*8.8478e-9;
t42 = t7.*8.8478e-9;
t43 = t8.*8.8478e-9;
t44 = t9.*8.8478e-9;
t31 = -t28;
t32 = -t29;
t33 = -t30;
t34 = t10+t14+t15+t16;
t36 = t2+t4+t12+t13+t19+t20;
t37 = t4+t5+t12+t17+t18+t19;
t39 = -v_x.*(t2+t3-t4-t12+t13+t20);
t40 = v_x.*(t2+t3-t4-t12+t13+t20);
t35 = t34.*v_z;
t38 = t36.*v_y;
t45 = t35+t38+t40;
t46 = t45.^2;
t47 = t46.*6.5e-10;
t48 = t41+t42+t43+t44+t47;
v_dot = [F_ext_x.*(2.0e+1./1.3e+1)-v_x.*(t27+t28+t32+t33).*(2.0e+1./1.3e+1)-v_z.*(t22+t25).*(2.0e+1./1.3e+1)+v_y.*(t23-t24).*(2.0e+1./1.3e+1)+q_w.*q_y.*t48.*(4.0e+1./1.3e+1)+q_x.*q_z.*t48.*(4.0e+1./1.3e+1);F_ext_y.*(2.0e+1./1.3e+1)-v_y.*(t27+t29+t31+t33).*(2.0e+1./1.3e+1)-v_x.*(t23+t24).*(2.0e+1./1.3e+1)+v_z.*(t21-t26).*(2.0e+1./1.3e+1)-q_w.*q_x.*t48.*(4.0e+1./1.3e+1)+q_y.*q_z.*t48.*(4.0e+1./1.3e+1);F_ext_z.*(2.0e+1./1.3e+1)-v_z.*(t27+t30+t31+t32).*(2.0e+1./1.3e+1)-v_y.*(t21+t26).*(2.0e+1./1.3e+1)+t2.*t48.*(2.0e+1./1.3e+1)-t3.*t48.*(2.0e+1./1.3e+1)-t4.*t48.*(2.0e+1./1.3e+1)+t5.*t48.*(2.0e+1./1.3e+1)+v_x.*(t22-t25).*(2.0e+1./1.3e+1)-9.81e+2./1.0e+2];