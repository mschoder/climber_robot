function gam_sol = gam_solved_climber(in1,in2)
%GAM_SOLVED_CLIMBER
%    GAM_SOL = GAM_SOLVED_CLIMBER(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    19-Nov-2020 21:05:51

L_AB = in2(7,:);
L_BC = in2(8,:);
L_CE = in2(11,:);
L_EG = in2(12,:);
L_GH = in2(16,:);
delta = in2(6,:);
dth2 = in1(5,:);
th2 = in1(2,:);
t2 = cos(delta);
t3 = sin(delta);
t4 = cos(th2);
t5 = sin(th2);
t6 = delta+th2;
t7 = L_AB.^2;
t8 = L_BC.^2;
t9 = L_CE.^2;
t10 = L_EG.^2;
t11 = L_GH.^2;
t17 = L_BC.*L_CE.*2.0;
t18 = L_BC.*L_GH.*2.0;
t19 = L_CE.*L_GH.*2.0;
t12 = L_AB.*t2;
t13 = L_AB.*t3;
t14 = L_EG.*t4;
t15 = L_EG.*t5;
t16 = cos(t6);
t20 = -t12;
t21 = L_BC.*t12.*2.0;
t22 = L_CE.*t12.*2.0;
t23 = L_GH.*t12.*2.0;
t24 = L_BC.*t14.*2.0;
t25 = L_CE.*t14.*2.0;
t26 = L_GH.*t14.*2.0;
t30 = L_AB.*L_EG.*t16.*2.0;
t31 = t13+t15;
t27 = -t21;
t28 = -t22;
t29 = -t23;
t32 = -t30;
t33 = 1.0./t31;
t34 = t33.^2;
t35 = t7+t8+t9+t10+t11+t17+t18+t19+t24+t25+t26+t27+t28+t29+t32;
t36 = sqrt(t35);
t37 = conj(t36);
t38 = -t37;
t39 = L_BC+L_CE+L_GH+t14+t20+t38;
gam_sol = [conj(atan(t33.*(L_BC+L_CE+L_GH+t14+t20-t36))).*2.0;(dth2.*(t33.*(t15-(L_EG.*(L_BC.*t5+L_CE.*t5+L_GH.*t5-L_AB.*sin(t6)))./t37)+t14.*t34.*t39).*-2.0)./(t34.*t39.^2+1.0)];
