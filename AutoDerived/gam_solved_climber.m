function gam_sol = gam_solved_climber(in1,in2)
%GAM_SOLVED_CLIMBER
%    GAM_SOL = GAM_SOLVED_CLIMBER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    16-Nov-2020 15:44:41

L_AB = in2(7,:);
L_BC = in2(8,:);
L_CE = in2(11,:);
L_EG = in2(12,:);
L_GH = in2(16,:);
delta = in2(6,:);
dth1 = in1(6,:);
dth2 = in1(7,:);
th1 = in1(2,:);
th2 = in1(3,:);
t2 = cos(delta);
t3 = cos(th1);
t4 = cos(th2);
t5 = sin(th1);
t6 = sin(th2);
t7 = delta+th1;
t8 = th1+th2;
t9 = L_AB.^2;
t10 = L_BC.^2;
t11 = L_CE.^2;
t12 = L_EG.^2;
t13 = L_GH.^2;
t23 = L_CE.*L_GH.*2.0;
t14 = L_BC.*t3;
t15 = L_EG.*t4;
t16 = L_BC.*t5;
t17 = L_EG.*t6;
t18 = cos(t7);
t19 = sin(t7);
t20 = cos(t8);
t21 = t7+th2;
t22 = sin(t8);
t29 = L_AB.*L_BC.*t2.*2.0;
t24 = cos(t21);
t25 = sin(t21);
t26 = L_AB.*t18;
t27 = L_AB.*t19;
t28 = -t16;
t30 = L_CE.*t14.*2.0;
t31 = L_CE.*t15.*2.0;
t32 = L_GH.*t14.*2.0;
t33 = L_GH.*t15.*2.0;
t34 = -t29;
t38 = L_BC.*L_EG.*t20.*2.0;
t35 = -t26;
t36 = L_CE.*t26.*2.0;
t37 = L_GH.*t26.*2.0;
t39 = L_AB.*L_EG.*t24.*2.0;
t43 = t17+t27+t28;
t40 = -t36;
t41 = -t37;
t42 = -t39;
t44 = 1.0./t43;
t45 = t44.^2;
t46 = t9+t10+t11+t12+t13+t23+t30+t31+t32+t33+t34+t38+t40+t41+t42;
t47 = sqrt(t46);
t48 = conj(t47);
t49 = -t48;
t50 = 1.0./t48;
t51 = L_CE+L_GH+t14+t15+t35+t49;
t52 = t51.^2;
t53 = t45.*t52;
t54 = t53+1.0;
t55 = 1.0./t54;
gam_sol = [conj(atan(t44.*(L_CE+L_GH+t14+t15+t35-t47))).*-2.0;dth1.*t55.*(t44.*(t27+t28+t50.*(L_CE.*t16-L_CE.*t27+L_GH.*t16-L_GH.*t27-L_AB.*L_EG.*t25+L_BC.*L_EG.*t22))+t45.*t51.*(t14+t35)).*-2.0+dth2.*t55.*(t44.*(t17-L_EG.*t50.*(-L_AB.*t25+L_BC.*t22+L_CE.*t6+L_GH.*t6))+t15.*t45.*t51).*2.0];
