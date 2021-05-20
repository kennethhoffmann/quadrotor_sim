function con = motor_con(in1,in2,in3)
%MOTOR_CON
%    CON = MOTOR_CON(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    18-May-2021 18:45:45

u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
u_p2 = in3(2,:);
u_p3 = in3(3,:);
u_p4 = in3(4,:);
t2 = u1.*2.82556115644567e+7;
t3 = u2.*4.916795684662522e+7;
t4 = u3.*4.916795684662522e+7;
t5 = u_p2.*4.916795684662522e+7;
t6 = u_p3.*4.916795684662522e+7;
t7 = u4.*7.092158502678632e+7;
t8 = u_p4.*7.092158502678632e+7;
t15 = u2.*u4.*3.096942736443277e+5;
t16 = u3.*u4.*3.096942736443277e+5;
t9 = -t3;
t10 = -t4;
t11 = -t5;
t12 = -t6;
t13 = -t7;
t14 = -t8;
t17 = -t15;
t18 = -t16;
t19 = t2+t3+t6+t7+t10+t11+t14+t15+t16;
t20 = t2+t5+t6+t8+t9+t10+t13+t15+t18;
t21 = t2+t3+t4+t8+t11+t12+t13+t16+t17;
t22 = t2+t4+t5+t7+t9+t12+t14+t17+t18;
t23 = sqrt(t19);
t24 = sqrt(t20);
t25 = sqrt(t21);
t26 = sqrt(t22);
con = [t24-3.3e+4;t25-3.3e+4;t23-3.3e+4;t26-3.3e+4;-t24+8.0e+2;-t25+8.0e+2;-t23+8.0e+2;-t26+8.0e+2];
