function con_cost = con_cost(in1,in2,in3)
%CON_COST
%    CON_COST = CON_COST(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    03-Jun-2021 20:37:04

con1 = in1(1,:);
con2 = in1(2,:);
con3 = in1(3,:);
con4 = in1(4,:);
con5 = in1(5,:);
con6 = in1(6,:);
con7 = in1(7,:);
con8 = in1(8,:);
con9 = in1(9,:);
con10 = in1(10,:);
con11 = in1(11,:);
con12 = in1(12,:);
con13 = in1(13,:);
con14 = in1(14,:);
con15 = in1(15,:);
con16 = in1(16,:);
con17 = in1(17,:);
con18 = in1(18,:);
con19 = in1(19,:);
con20 = in1(20,:);
con21 = in1(21,:);
con22 = in1(22,:);
con23 = in1(23,:);
con24 = in1(24,:);
lambda1 = in2(1,:);
lambda2 = in2(2,:);
lambda3 = in2(3,:);
lambda4 = in2(4,:);
lambda5 = in2(5,:);
lambda6 = in2(6,:);
lambda7 = in2(7,:);
lambda8 = in2(8,:);
lambda9 = in2(9,:);
lambda10 = in2(10,:);
lambda11 = in2(11,:);
lambda12 = in2(12,:);
lambda13 = in2(13,:);
lambda14 = in2(14,:);
lambda15 = in2(15,:);
lambda16 = in2(16,:);
lambda17 = in2(17,:);
lambda18 = in2(18,:);
lambda19 = in2(19,:);
lambda20 = in2(20,:);
lambda21 = in2(21,:);
lambda22 = in2(22,:);
lambda23 = in2(23,:);
lambda24 = in2(24,:);
mu_diag1 = in3(1,:);
mu_diag2 = in3(2,:);
mu_diag3 = in3(3,:);
mu_diag4 = in3(4,:);
mu_diag5 = in3(5,:);
mu_diag6 = in3(6,:);
mu_diag7 = in3(7,:);
mu_diag8 = in3(8,:);
mu_diag9 = in3(9,:);
mu_diag10 = in3(10,:);
mu_diag11 = in3(11,:);
mu_diag12 = in3(12,:);
mu_diag13 = in3(13,:);
mu_diag14 = in3(14,:);
mu_diag15 = in3(15,:);
mu_diag16 = in3(16,:);
mu_diag17 = in3(17,:);
mu_diag18 = in3(18,:);
mu_diag19 = in3(19,:);
mu_diag20 = in3(20,:);
mu_diag21 = in3(21,:);
mu_diag22 = in3(22,:);
mu_diag23 = in3(23,:);
mu_diag24 = in3(24,:);
con_cost = con1.*(lambda1+(con1.*mu_diag1)./2.0)+con2.*(lambda2+(con2.*mu_diag2)./2.0)+con3.*(lambda3+(con3.*mu_diag3)./2.0)+con4.*(lambda4+(con4.*mu_diag4)./2.0)+con5.*(lambda5+(con5.*mu_diag5)./2.0)+con6.*(lambda6+(con6.*mu_diag6)./2.0)+con7.*(lambda7+(con7.*mu_diag7)./2.0)+con8.*(lambda8+(con8.*mu_diag8)./2.0)+con9.*(lambda9+(con9.*mu_diag9)./2.0)+con10.*(lambda10+(con10.*mu_diag10)./2.0)+con11.*(lambda11+(con11.*mu_diag11)./2.0)+con12.*(lambda12+(con12.*mu_diag12)./2.0)+con13.*(lambda13+(con13.*mu_diag13)./2.0)+con14.*(lambda14+(con14.*mu_diag14)./2.0)+con15.*(lambda15+(con15.*mu_diag15)./2.0)+con16.*(lambda16+(con16.*mu_diag16)./2.0)+con17.*(lambda17+(con17.*mu_diag17)./2.0)+con18.*(lambda18+(con18.*mu_diag18)./2.0)+con19.*(lambda19+(con19.*mu_diag19)./2.0)+con20.*(lambda20+(con20.*mu_diag20)./2.0)+con21.*(lambda21+(con21.*mu_diag21)./2.0)+con22.*(lambda22+(con22.*mu_diag22)./2.0)+con23.*(lambda23+(con23.*mu_diag23)./2.0)+con24.*(lambda24+(con24.*mu_diag24)./2.0);
