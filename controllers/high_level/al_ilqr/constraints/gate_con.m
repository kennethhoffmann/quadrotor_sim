function con = gate_con(in1,in2)
%GATE_CON
%    CON = GATE_CON(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Jun-2021 21:59:52

x2 = in1(2,:);
x3 = in1(3,:);
x7 = in1(7,:);
x8 = in1(8,:);
x9 = in1(9,:);
x10 = in1(10,:);
t2 = x3.*5.0;
t3 = x7.^2;
t4 = x8.^2;
t5 = x9.^2;
t6 = x10.^2;
t8 = x2.*(2.5e+1./6.0);
t10 = x7.*x10.*(5.9e+1./8.0e+1);
t11 = x8.*x9.*(5.9e+1./8.0e+1);
t18 = x7.*x8.*(1.77e+2./2.0e+2);
t19 = x7.*x9.*(1.77e+2./2.0e+2);
t20 = x8.*x10.*(1.77e+2./2.0e+2);
t21 = x9.*x10.*(1.77e+2./2.0e+2);
t7 = -t2;
t9 = -t8;
t12 = t3.*(5.9e+1./1.6e+2);
t13 = t4.*(5.9e+1./1.6e+2);
t14 = t5.*(5.9e+1./1.6e+2);
t15 = t6.*(5.9e+1./1.6e+2);
t16 = -t10;
t17 = -t11;
t26 = -t18;
t27 = -t19;
t28 = -t20;
t29 = -t21;
t22 = -t12;
t23 = -t13;
t24 = -t14;
t25 = -t15;
con = [t7+t19+t28+1.9e+1./2.0;t7+t18+t21+1.9e+1./2.0;t7+t20+t27+1.9e+1./2.0;t7+t26+t29+1.9e+1./2.0;t9+t16+t17-1.4e+1./3.0;t9+t12+t14+t23+t25-1.4e+1./3.0;t9+t10+t11-1.4e+1./3.0;t9+t13+t15+t22+t24-1.4e+1./3.0;t2+t20+t27-2.1e+1./2.0;t2+t26+t29-2.1e+1./2.0;t2+t19+t28-2.1e+1./2.0;t2+t18+t21-2.1e+1./2.0;t8+t10+t11+1.1e+1./3.0;t8+t13+t15+t22+t24+1.1e+1./3.0;t8+t16+t17+1.1e+1./3.0;t8+t12+t14+t23+t25+1.1e+1./3.0];
