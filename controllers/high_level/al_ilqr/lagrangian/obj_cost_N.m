function obj_cost_N = obj_cost_N(in1,in2,in3,in4)
%OBJ_COST_N
%    OBJ_COST_N = OBJ_COST_N(IN1,IN2,IN3,IN4)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Jun-2021 21:57:48

x1 = in1(1,:);
x2 = in1(2,:);
x3 = in1(3,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
x_bar1 = in3(1,:);
x_bar2 = in3(2,:);
x_bar3 = in3(3,:);
x_bar4 = in3(4,:);
x_bar5 = in3(5,:);
x_bar6 = in3(6,:);
x_bar7 = in3(7,:);
x_bar8 = in3(8,:);
x_bar9 = in3(9,:);
x_bar10 = in3(10,:);
t2 = -x_bar1;
t3 = -x_bar2;
t4 = -x_bar3;
t5 = -x_bar4;
t6 = -x_bar5;
t7 = -x_bar6;
t8 = -x_bar7;
t9 = -x_bar8;
t10 = -x_bar9;
t11 = -x_bar10;
t12 = x_bar1-2.0;
t13 = x_bar3-1.0;
t14 = x_bar7-1.0;
t15 = t2+x1;
t16 = t3+x2;
t17 = t4+x3;
t18 = t5+x4;
t19 = t6+x5;
t20 = t7+x6;
t21 = t8+x7;
t22 = t9+x8;
t23 = t10+x9;
t24 = t11+x10;
obj_cost_N = t12.*(x_bar1./2.0-1.0)+t13.*(x_bar3./2.0-1.0./2.0)+t14.*(x_bar7./2.0-1.0./2.0)+t12.*t15+t13.*t17+t14.*t21+t16.*x_bar2+t18.*x_bar4+t19.*x_bar5+t20.*x_bar6+t22.*x_bar8+t23.*x_bar9+t24.*x_bar10+t15.^2+t16.^2+t17.^2+t18.^2+t19.^2+t20.^2+t21.^2+t22.^2+t23.^2+t24.^2+x_bar2.^2./2.0+x_bar4.^2./2.0+x_bar5.^2./2.0+x_bar6.^2./2.0+x_bar8.^2./2.0+x_bar9.^2./2.0+x_bar10.^2./2.0;
